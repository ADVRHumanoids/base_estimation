import casadi as cs
import numpy as np
from horizon import problem as csprb
import matplotlib as plt
import python.step_interpolator as stp_interp
import logging

def casadi_sum(x, axis=None, out=None):
    assert out is None
    if axis == 0:
        return cs.sum1(x)
    elif axis == 1:
        return cs.sum2(x)
    elif axis is None:
        return cs.sum1(cs.sum2(x))
    else:
        raise Exception("Invalid argument for sum")

def RK4(M, L, x, u, xdot, dt):
    """RK4 Runge-Kutta 4 integrator
    TODO: PUT IT IN ANOTHER CLASS
    Input:
        L: objective fuction to integrate
        M: RK steps
        T: final time
        N: numbr of shooting nodes:
        x: state varibales
        u: controls
    """


    f = cs.Function('f', [x, u], [xdot, L])
    X0 = cs.SX.sym('X0', x.size()[0])
    U = cs.SX.sym('U', u.size()[0])
    X = X0
    Q = 0

    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + dt / 2 * k1, U)
        k3, k3_q = f(X + dt / 2 * k2, U)
        k4, k4_q = f(X + dt * k3, U)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q = Q + dt / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

    return cs.Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

class StepSolver:

    def __init__(self, n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com):

        self.n_duration = n_duration

        self.initial_ds_t = initial_ds_t  # 0.2
        self.ss_1_t = single_stance_t  # 0.5
        self.ds_1_t = 0.
        self.ss_2_t = 0.
        self.final_ds_t = final_ds_t  # 0.4

        self.T_total = self.initial_ds_t + self.ss_1_t + self.ds_1_t + self.ss_2_t + self.final_ds_t
        self.N = int(self.T_total / self.n_duration)  # number of control intervals

        self.height_com = height_com

        print('duration of initial_ds_t: {}'.format(self.initial_ds_t))
        print('duration of ss_1_t: {}'.format(self.ss_1_t))
        print('duration of ds_1_t: {}'.format(self.ds_1_t))
        print('duration of ss_2_t: {}'.format(self.ss_2_t))
        print('duration of final_ds_t: {}'.format(self.final_ds_t))
        print('T: {}'.format(self.T_total))
        print('N: {}'.format(self.N))
        print('duration of a single node: {}'.format(self.n_duration))

        self.initial_ds_n = int(self.initial_ds_t / self.n_duration)
        self.ss_1_n = int(self.ss_1_t / self.n_duration)
        self.ds_1_n = int(self.ds_1_t / self.n_duration)
        self.ss_2_n = int(self.ss_2_t / self.n_duration)
        self.final_ds_n = int(self.final_ds_t / self.n_duration)

        # print('duration (in nodes) of initial ds: {}'.format(initial_ds_n))
        # print('duration (in nodes) of first ss: {}'.format(ss_1_n))
        # print('duration (in nodes) of middle ds: {}'.format(ds_1_n))
        # print('duration (in nodes) of second ss: {}'.format(ss_2_n))
        # print('duration (in nodes) of final ds: {}'.format(final_ds_n))

        self.ds_1 = self.initial_ds_n
        self.ss_1 = self.ds_1 + self.ss_1_n
        self.ds_2 = self.ss_1 + self.ds_1_n
        self.ss_2 = self.ds_2 + self.ss_2_n
        ds_3 = self.ss_2 + self.final_ds_n

        self.sym_c = cs.SX

        # state variables
        self.p_abst = self.sym_c.sym('p_abst', 2)  # com position
        self.v_abst = self.sym_c.sym('v_abst', 2)  # com velocity
        self.a_abst = self.sym_c.sym('a_abst', 2)  # com acceleration
        self.x_abst = cs.vertcat(self.p_abst, self.v_abst, self.a_abst)  # , l, r, alpha_l, alpha_r # state
        # control variables
        self.j_abst = self.sym_c.sym('j_abst', 2)  # com jerk
        self.u_abst = self.j_abst  # control
        # model equation
        self.xdot_abst = cs.vertcat(self.v_abst, self.a_abst, self.j_abst)

        # Objective terms
        self.L = cs.sumsqr(self.u_abst)
        # Formulate discrete time dynamics
        # Fixed step Runge-Kutta 4 integrator
        self.M = 1  # RK4 steps per interval
        self.dt = self.T_total / self.N / self.M

        margin = 0.0
        self.width_foot = 0.1 - margin
        self.length_foot = 0.2 - margin

        self.max_stride_x = 0.4
        self.max_stride_y = self.width_foot / 2. + 0.5 #0.3 #
        self.min_stride_y = self.width_foot / 2. + 0.15

        self.grav = 9.81

    def getEdges(self, p):  # lu, ru, rh, lh

        lu = cs.DM([+ self.length_foot / 2., + self.width_foot / 2.])
        ru = cs.DM([+ self.length_foot / 2., - self.width_foot / 2.])
        rh = cs.DM([- self.length_foot / 2., - self.width_foot / 2.])
        lh = cs.DM([- self.length_foot / 2., + self.width_foot / 2.])

        ft_vert = cs.horzcat(p + lu, p + ru, p + rh, p + lh).T

        return ft_vert

    def stepPattern(self, initial_n, final_n, type_leg):

        if type_leg == 'D':
            active_nodes = list(range(initial_n, final_n)) if initial_n != final_n else initial_n
            
            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_zmp_stab',
                                      self.zmp - (casadi_sum(self.wl_vert, 0).T + casadi_sum(self.wr_vert, 0).T),
                                      nodes=active_nodes, # [initial_n, final_n]
                                      bounds=dict(lb=[0., 0.], ub=[0., 0.]))

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_contacts',
                                      casadi_sum(self.alpha_l, 0).T + casadi_sum(self.alpha_r, 0).T,
                                      nodes=active_nodes, # [initial_n, final_n]
                                      bounds=dict(lb=[1.], ub=[1.]))
            if initial_n == 0:
                initial_n = 1

            active_nodes = list(range(initial_n, final_n)) if initial_n != final_n else initial_n
            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_l_foot',
                                      self.l - self.l_prev,
                                      nodes=active_nodes, # [initial_n, final_n]
                                      bounds=dict(lb=[0., 0.], ub=[0., 0.]))

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_r_foot',
                                      self.r - self.r_prev,
                                      nodes=active_nodes, # [initial_n, final_n]
                                      bounds=dict(lb=[0., 0.], ub=[0., 0.]))

        else:
            if type_leg.lower() == 'r':
                other_leg = self.l
                stance_w_vert = self.wl_vert
                stance_alpha = self.alpha_r
                swing_alpha = self.alpha_l
                fixed_foot = self.l
                fixed_foot_prev = self.l_prev
                stance_leg_string = 'l'


            else:
                other_leg = self.r
                stance_w_vert = self.wr_vert
                stance_alpha = self.alpha_l
                swing_alpha = self.alpha_r
                fixed_foot = self.r
                fixed_foot_prev = self.r_prev
                stance_leg_string = 'r'

            active_nodes = list(range(initial_n, final_n)) if initial_n != final_n else initial_n

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_zmp_stab',
                                      self.zmp - casadi_sum(stance_w_vert, 0).T,
                                      nodes=active_nodes, # [initial_n, final_n],
                                      bounds=dict(lb=[0., 0.], ub=[0., 0.]))

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_contacts_' + type_leg.lower(),
                                      casadi_sum(stance_alpha, 0).T,
                                      nodes=active_nodes, # [initial_n, final_n],
                                      bounds=dict(lb=[0.], ub=[0.]))

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_contacts_' + stance_leg_string,
                                      casadi_sum(swing_alpha, 0).T,
                                      nodes=active_nodes, # [initial_n, final_n],
                                      bounds=dict(lb=[1.], ub=[1.]))

            self.prb.createConstraint(type_leg + '_' + str(final_n) + '_r_foot',
                                      fixed_foot - fixed_foot_prev,
                                      nodes=active_nodes, # [initial_n, final_n],
                                      bounds=dict(lb=[0., 0.], ub=[0., 0.]))


    def buildProblemStep(self):

        self.prb = csprb.Problem(self.N, crash_if_suboptimal=True)  #logging_level=logging.DEBUG

        self.x = self.prb.createStateVariable('x', 6)
        self.x_prev = self.prb.createStateVariable('x', 6, -1)

        self.l = self.prb.createStateVariable('l', 2)
        self.r = self.prb.createStateVariable('r', 2)

        self.l_prev = self.prb.createStateVariable('l', 2, -1)
        self.r_prev = self.prb.createStateVariable('r', 2, -1)

        self.alpha_l = self.prb.createStateVariable('alpha_l', 4)
        self.alpha_r = self.prb.createStateVariable('alpha_r', 4)

        self.u = self.prb.createInputVariable('u', 2)
        self.u_prev = self.prb.createInputVariable('u', 2, -1)


        # zmp variable
        self.zmp = self.x[0:2] - self.x[4:6] * (self.height_com / self.grav)

        # integrator for multiple shooting
        integrator = RK4(self.M, self.L, self.x_abst, self.u_abst, self.xdot_abst, self.dt)
        self.x_int = integrator(x0=self.x_prev, p=self.u_prev)

        self.wl_vert = self.alpha_l * self.getEdges(self.l)
        self.wr_vert = self.alpha_r * self.getEdges(self.r)

        ds_n_1 = self.initial_ds_n + 1
        ss_n_1 = self.ds_1 + self.ss_1_n + 1
        ds_n_2 = self.ss_1 + self.ds_1_n + 1
        ss_n_2 = self.ds_2 + self.ss_2_n + 1
        ds_n_3 = self.N + 1

        # todo remember that the last node is N+1! change something
        # add constraints

        multi_shoot = self.prb.createConstraint('multiple_shooting',
                                                self.x_int['xf'] - self.x,
                                                nodes=list(range(1, self.N + 1)), #[1, self.N + 1],
                                                bounds=dict(ub=[0., 0., 0., 0., 0., 0.], lb=[0., 0., 0., 0., 0., 0.]))


        self.stepPattern(     0, ds_n_1, 'D')
        self.stepPattern(ds_n_1, ss_n_1, 'L')
        self.stepPattern(ss_n_1, ds_n_2, 'D') # this is zero right now
        self.stepPattern(ds_n_2, ss_n_2, 'R') # this is zero right now
        self.stepPattern(ss_n_2, ds_n_3, 'D')

        # add cost functions
        # self.prb.setCostFunction('minimize_input', 0.001 * cs.sumsqr(prb_vars['u']), nodes=[0, self.N]) # todo trim to lenght of specific node (here 'u')
        self.prb.createCostFunction('minimize_l_motion', cs.sumsqr(self.l - self.l_prev), nodes=list(range(1, self.N+1)) )#[1, self.N + 1],)
        self.prb.createCostFunction('minimize_r_motion', cs.sumsqr(self.r - self.r_prev), nodes=list(range(1, self.N+1)) )#[1, self.N + 1],)

        self.prb.createProblem()

    def solveProblemStep(self, initial_com, initial_l_foot, initial_r_foot):

        initial_lbw_com = [initial_com[0, 0], initial_com[0, 1],  # com pos
                           initial_com[1, 0], initial_com[1, 1],  # com vel
                           initial_com[2, 0], initial_com[2, 1]]

        initial_ubw_com = [initial_com[0, 0], initial_com[0, 1],
                           initial_com[1, 0], initial_com[1, 1],
                           initial_com[2, 0], initial_com[2, 1]]

        final_lbw_com = [-cs.inf, -cs.inf,
                         0.0, 0.0,
                         0.0, 0.0]

        final_ubw_com = [cs.inf, cs.inf,
                         0.0, 0.0,
                         0.0, 0.0]

        # todo check if lenght of ubw and lbw are of the right size
        self.x.setBounds(nodes=0, lb=initial_lbw_com, ub=initial_ubw_com)
        self.l.setBounds(nodes=0, lb=[initial_l_foot[0], initial_l_foot[1]], ub=[initial_l_foot[0], initial_l_foot[1]])
        self.r.setBounds(nodes=0, lb=[initial_r_foot[0], initial_r_foot[1]], ub=[initial_r_foot[0], initial_r_foot[1]])

        self.x.setBounds(nodes=self.N, lb=final_lbw_com, ub=final_ubw_com)

        self.alpha_l.setBounds(lb=[0., 0., 0., 0.], ub=[1., 1., 1., 1.])
        self.alpha_r.setBounds(lb=[0., 0., 0., 0.], ub=[1., 1., 1., 1.])

        self.u.setBounds(nodes=list(range(self.N)) , lb=[-1000., -1000.], ub=[1000., 1000.]) #nodes=[0, self.N]

        w_opt = self.prb.solveProblem()

        return w_opt

    # todo UP TO HERE IT IS OK!

    def interpolateStep(self, initial_com, opt_values, t_i, t_f, freq):

        # ----------------------------------------------------------
        # interpolate the CoM at a higher frequency (from 0.2 to 0.1)
        com_interpol = cs.DM(initial_com.flatten())
        multiplier = 2
        dt = self.n_duration
        dt_int = dt / 1 / multiplier
        # pos vel acc

        integrator = RK4(self.M, self.L, self.x_abst, self.u_abst, self.xdot_abst, dt_int)
        # ----------------------------------------------------------
        # interpolate CoM using initial_com as x_0 and the optimal input found before, but with a frequency multiplied
        # apply *multiplier* time the input at every loop
        for i in range(opt_values['u'].shape[1]):
            for multi_i in range(multiplier):
                Fk = integrator(x0=com_interpol[:, -1], p=opt_values['u'][:, i])
                com_interpol = cs.horzcat(com_interpol, Fk['xf'])

        com_traj = dict()
        com_traj['x'] = com_interpol[0, :].full().flatten()  # pos x
        com_traj['y'] = com_interpol[1, :].full().flatten()  # pos y
        com_traj['dx'] = com_interpol[2, :].full().flatten()  # vel x
        com_traj['dy'] = com_interpol[3, :].full().flatten()  # vel y

        relevant_nodes = [0, self.ds_2 + self.ss_2_n + 1]
        # "manual interpolator
        t, l_foot_traj = stp_interp.interpolator(opt_values['l'],
                                      step_i=opt_values['l'][:, relevant_nodes[0]],
                                      step_f=opt_values['l'][:, relevant_nodes[1]], step_height=0.05,
                                      time=self.T_total,
                                      t_i=t_i, t_f=t_f, freq=freq)

        return com_traj, l_foot_traj

#
#     def homing(duration):
#
#         # get x of robot
#         x_l_foot = model.getPose(ctrl_tasks[0].getName()).translation[0]
#         x_r_foot = model.getPose(ctrl_tasks[1].getName()).translation[0]
#
#         print('x_r_foot', x_r_foot)
#
#         com_initial = model.getCOM()
#
#         com_correction = -com_initial[0] + x_r_foot
#         print('com_correction: ', com_correction)
#
#         # command com to reach feet x
#
#         goal_com = Affine3()
#         goal_com.translation = com_initial
#         print('goal_com.translation', goal_com.translation[0])
#         goal_com.translation[0] += com_correction
#
#         ci_solver.reach(com_task, goal_com, duration, sim=1)

