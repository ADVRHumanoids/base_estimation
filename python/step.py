# /usr/bin/env python3

from horizon import Problem
from horizon import RK4
from horizon import casadi_sum
from horizon import interpolator

import time

from com_vel_est import FloatingBase
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np
import casadi as cs

##### for robot and stuff
import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from ci_solver import CartesianInterfaceSolver

class StepSolver:

    def __init__(self, n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com):

        self.n_duration = n_duration

        self.initial_ds_t = initial_ds_t  # 0.2
        self.ss_1_t = single_stance_t  # 0.5
        self.ds_1_t = 0.
        self.ss_2_t = 0.
        self.final_ds_t = final_ds_t  # 0.4

        self.T_total = initial_ds_t + self.ss_1_t + self.ds_1_t + self.ss_2_t + self.final_ds_t
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
        self.p = self.sym_c.sym('p', 2)  # com position
        self.v = self.sym_c.sym('v', 2)  # com velocity
        self.a = self.sym_c.sym('a', 2)  # com acceleration
        self.x = cs.vertcat(self.p, self.v, self.a)  # , l, r, alpha_l, alpha_r # state
        # control variables
        self.j = self.sym_c.sym('j', 2)  # com jerk
        self.u = self.j  # control
        # model equation
        self.xdot = cs.vertcat(self.v, self.a, self.j)

        # Objective terms
        self.L = cs.sumsqr(self.u)
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

    def buildProblemStep(self):

        self.prb = Problem(self.N)

        self.prb.setVariable('x', 6)
        self.prb.setVariable('x-1', 6)

        self.prb.setVariable('l', 2)
        self.prb.setVariable('r', 2)

        self.prb.setVariable('l-1', 2)
        self.prb.setVariable('r-1', 2)

        self.prb.setVariable('alpha_l', 4)
        self.prb.setVariable('alpha_r', 4)

        self.prb.setVariable('u', 2)
        self.prb.setVariable('u-1', 2)
        prb_vars = self.prb.getVariable()

        # zmp variable
        zmp = prb_vars['x'][0:2] - prb_vars['x'][4:6] * (self.height_com / self.grav)
        self.prb.setFunction('zmp', zmp)

        # integrator for multiple shooting
        integrator = RK4(self.M, self.L, self.x, self.u, self.xdot, self.dt)
        x_int = integrator(x0=prb_vars['x-1'], p=prb_vars['u-1'])

        self.prb.setFunction('x_int', x_int)

        wl_vert = prb_vars['alpha_l'] * self.getEdges(prb_vars['l'])
        wr_vert = prb_vars['alpha_r'] * self.getEdges(prb_vars['r'])

        self.prb.setFunction('wl_vert', wl_vert)
        self.prb.setFunction('wr_vert', wr_vert)

        prb_funs = self.prb.getFunction()

        ds_n_1 = self.initial_ds_n + 1
        ss_n_1 = self.ds_1 + self.ss_1_n + 1
        ds_n_2 = self.ss_1 + self.ds_1_n + 1
        ss_n_2 = self.ds_2 + self.ss_2_n + 1

        # todo remember that the last node is N+1! change something
        # add constraints
        self.prb.ct.setConstraintFunction('multiple_shooting', prb_funs['x_int']['xf'] - prb_vars['x'], nodes=[1, self.N + 1],
                                     bounds=dict(ubg=[0., 0., 0., 0., 0., 0.], lbg=[0., 0., 0., 0., 0., 0.]))
        self.prb.ct.setConstraintFunction('stride', prb_vars['l'] - prb_vars['r'],
                                     bounds=dict(lbg=[-self.max_stride_x, -self.max_stride_y], ubg=[self.max_stride_x, self.max_stride_y]))

        # # double stance
        self.prb.ct.setConstraintFunction('ds1_zmp_stab', prb_funs['zmp'] - (
                    casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T),
                                     nodes=[0, ds_n_1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds1_contacts',
                                     casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T,
                                     nodes=[0, ds_n_1], bounds=dict(lbg=[1.], ubg=[1.]))
        self.prb.ct.setConstraintFunction('ds1_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[1, ds_n_1],
                                     bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds1_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[1, ds_n_1],
                                     bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        #
        # #single
        self.prb.ct.setConstraintFunction('ss1_zmp_stab', prb_funs['zmp'] - casadi_sum(prb_funs['wr_vert'], 0).T,
                                     nodes=[ds_n_1, ss_n_1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ss1_contacts_l', casadi_sum(prb_vars['alpha_l'], 0).T,
                                     nodes=[ds_n_1, ss_n_1], bounds=dict(lbg=[0.], ubg=[0.]))
        self.prb.ct.setConstraintFunction('ss1_contacts_r', casadi_sum(prb_vars['alpha_r'], 0).T,
                                     nodes=[ds_n_1, ss_n_1], bounds=dict(lbg=[1.], ubg=[1.]))
        self.prb.ct.setConstraintFunction('ss1_r_foot', prb_vars['r'] - prb_vars['r-1'],
                                     nodes=[ds_n_1, ss_n_1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

        # # double stance ds_2 + self.ss_2_n
        self.prb.ct.setConstraintFunction('ds2_zmp_stab', prb_funs['zmp'] - (
                    casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T),
                                     nodes=[ss_n_1, ds_n_2], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds2_contacts',
                                     casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T,
                                     nodes=[ss_n_1, ds_n_2], bounds=dict(lbg=[1.], ubg=[1.]))
        self.prb.ct.setConstraintFunction('ds2_l_foot', prb_vars['l'] - prb_vars['l-1'],
                                     nodes=[ss_n_1, ds_n_2], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds2_r_foot', prb_vars['r'] - prb_vars['r-1'],
                                     nodes=[ss_n_1, ds_n_2], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        #
        # #single stance
        self.prb.ct.setConstraintFunction('ss2_zmp_stab', prb_funs['zmp'] - casadi_sum(prb_funs['wl_vert'], 0).T,
                                     nodes=[ds_n_2, ss_n_2], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ss2_contacts_l', casadi_sum(prb_vars['alpha_l'], 0).T,
                                     nodes=[ds_n_2, ss_n_2], bounds=dict(lbg=[1.], ubg=[1.]))
        self.prb.ct.setConstraintFunction('ss2_contacts_r', casadi_sum(prb_vars['alpha_r'], 0).T,
                                     nodes=[ds_n_2, ss_n_2], bounds=dict(lbg=[0.], ubg=[0.]))
        self.prb.ct.setConstraintFunction('ss2_l_foot', prb_vars['l'] - prb_vars['l-1'],
                                     nodes=[ds_n_2, ss_n_2], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        #
        # # double stance
        self.prb.ct.setConstraintFunction('ds3_zmp_stab', prb_funs['zmp'] - (
                    casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T),
                                     nodes=[ss_n_2, self.N + 1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds3_contacts',
                                     casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T,
                                     nodes=[ss_n_2, self.N + 1], bounds=dict(lbg=[1.], ubg=[1.]))
        self.prb.ct.setConstraintFunction('ds3_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[ss_n_2, self.N + 1],
                                     bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
        self.prb.ct.setConstraintFunction('ds3_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[ss_n_2, self.N + 1],
                                     bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

        # add cost functions
        self.prb.setCostFunction('minimize_input', 0.001 * cs.sumsqr(prb_vars['u']), nodes=[0, self.N]) # todo trim to lenght of specific node (here 'u')
        self.prb.setCostFunction('minimize_velocity', cs.sumsqr(prb_vars['x'][2:4]))
        self.prb.setCostFunction('minimize_alpha', cs.sumsqr(prb_vars['alpha_l']) + cs.sumsqr(prb_vars['alpha_r']))
        self.prb.setCostFunction('minimize_stride_y', 1000. * cs.sumsqr((prb_vars['l'][1] - prb_vars['r'][1]) - self.min_stride_y))

        self.w, self.g = self.prb.buildProblem()

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

        self.prb.setStateBoundsFromName('x', nodes=0, lbw=initial_lbw_com, ubw=initial_ubw_com)

        self.prb.setStateBoundsFromName('l', nodes=0, lbw=[initial_l_foot[0], initial_l_foot[1]],
                                         ubw=[initial_l_foot[0], initial_l_foot[1]])
        self.prb.setStateBoundsFromName('r', nodes=0, lbw=[initial_r_foot[0], initial_r_foot[1]],
                                        ubw=[initial_r_foot[0], initial_r_foot[1]])

        self.prb.setStateBoundsFromName('x', nodes= self.N, lbw=final_lbw_com, ubw=final_ubw_com)

        self.prb.setStateBoundsFromName('alpha_l', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])
        self.prb.setStateBoundsFromName('alpha_r', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])

        self.prb.setStateBoundsFromName('u', nodes=[0, self.N + 1], lbw=[-1000., -1000.], ubw=[1000., 1000.])


        w_opt = self.prb.solveProblem()
        opt_values = self.prb.getOptimizedVariables(w_opt)

        return opt_values

    def interpolateStep(self, opt_values, freq):

        # ----------------------------------------------------------
        # interpolate the CoM at a higher frequency (from 0.2 to 0.1)
        com_interpol = cs.DM(initial_com.flatten())
        multiplier = 2
        dt = self.n_duration
        dt_int = dt / 1 / multiplier
        # pos vel acc
        integrator = RK4(self.M, self.L, self.x, self.u, self.xdot, dt_int)
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
        t, l_foot_traj = interpolator(step_i=opt_values['l'][:, relevant_nodes[0]],
                                      step_f=opt_values['l'][:, relevant_nodes[1]], step_height=0.05, time=self.T_total,
                                      t_i=initial_ds_t, t_f=initial_ds_t + self.ss_1_t, freq=freq)

        return com_traj, l_foot_traj


def tryWithoutRobot(n_duration, initial_ds_t, single_stance_t, final_ds_t):

    height_com = 0.911506523119
    solver = StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
    solver.buildProblemStep()

    initial_l_foot = np.array([-0.128, 0.103, 0.])
    initial_r_foot = np.array([-0.128, -0.103, 0.])
    initial_com = np.array([[-0.067, 0.], [0., 0.], [0., 0.]])

    opt_values = solver.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)

    # PLOTTING =============================================
    plt.figure()
    plt.clf()
    #
    # ######## COM ######
    plt.plot(opt_values['p'][0], opt_values['p'][1])

    # ######## ZMP ######
    zmp = list(np.array(opt_values['p']) - np.array(opt_values['a']) * (height_com / solver.grav))
    plt.plot(zmp[0], zmp[1], '-')

    alphal = np.sum(opt_values['alpha_l'], 0)
    alphar = np.sum(opt_values['alpha_r'], 0)
    ######## LEFT foot ######
    k = 0
    for pos_x, pos_y in zip(opt_values['l'][0], opt_values['l'][1]):
        if alphal[k] > 1e-6:
            coord = solver.getEdges([pos_x, pos_y])
            coord = cs.vertcat(coord, coord[0, :])
            plt.plot(coord[:, 0], coord[:, 1], color='b')
            plt.scatter(pos_x, pos_y)
        k += 1

    ######## RIGHT foot ######
    k = 0
    for pos_x, pos_y in zip(opt_values['r'][0], opt_values['r'][1]):
        if alphar[k].any() > 1e-6:
            coord = solver.getEdges([pos_x, pos_y])
            coord = cs.vertcat(coord, coord[0, :])
            plt.plot(coord[:, 0], coord[:, 1], color='r')
            plt.scatter(pos_x, pos_y)
        k += 1

    plt.show()
    exit()

if __name__ == '__main__':


    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('step')
    roscpp_init('step', [])

    ## build optimal problem
    initial_ds_t = 0.2
    single_stance_t = 0.5
    final_ds_t = 0.4
    T_tot = initial_ds_t + single_stance_t + final_ds_t

    n_duration = 0.02
    freq = 100.

    tryWithoutRobot(n_duration, initial_ds_t, single_stance_t, final_ds_t)

    ## PREPARE ROBOT
    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('/xbotcore/robot_description')
    srdf = rospy.get_param('/xbotcore/robot_description_semantic')

    opt = co.ConfigOptions()
    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_string_parameter('framework', 'ROS')
    model = xbot.ModelInterface(opt)
    robot = xbot.RobotInterface(opt)

    fb = FloatingBase()

    robot.sense()
    model.syncFrom(robot)
    model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
    model.update()

    # CREATE CARTESIAN SOLVER
    ctrl_points = {0: 'l_sole', 1: 'r_sole'}
    ci_solver = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=1./freq, ctrl_points=ctrl_points)
    print('Created cartesian interface.')

    ctrl_tasks, com_task = ci_solver.getTasks()

    height_com = model.getCOM()[2] - model.getPose(ctrl_tasks[0].getName()).translation[2]

    ## CONSTRUCT OPTIMAL PROBLEM
    solver = StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
    solver.buildProblemStep()


    com_task.setActivationState(pyci.ActivationState.Enabled)
    ctrl_tasks[0].setActivationState(pyci.ActivationState.Enabled)

    lfoot = Affine3()
    dlfoot = np.zeros([6, 1])
    #
    com = Affine3()
    com.translation = model.getCOM()
    dcom = np.zeros([6, 1])

    initial_t = 0
    t = 0
    i = 0
    rate = rospy.Rate(freq)


    i = 0
    threshold = 0.2
    flag_step = False

    t_start_step = None

    n_solve = 0
    max_solves = 1

    l_foot_initial = Affine3()
    r_foot_initial = Affine3()

    print('initial_com:', model.getCOM())
    print('started spinning...')
    while 1:

        # model.syncFrom(robot)
        model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
        model.update()

        # print(model.getCOM)

        if model.getCOMVelocity()[0] >= threshold and not flag_step and n_solve < max_solves:

            initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [0.3, 0], [0., 0.]])
            # initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [model.getCOMVelocity()[0], model.getCOMVelocity()[1]], [0., 0.]])

            l_foot_initial.translation = model.getPose(ctrl_tasks[0].getName()).translation
            l_foot_initial.linear = model.getPose(ctrl_tasks[0].getName()).linear

            r_foot_initial.translation = model.getPose(ctrl_tasks[1].getName()).translation
            r_foot_initial.linear = model.getPose(ctrl_tasks[1].getName()).linear

            initial_l_foot = np.array([l_foot_initial.translation[0], l_foot_initial.translation[1], 0.])
            initial_r_foot = np.array([r_foot_initial.translation[0], r_foot_initial.translation[1], 0.])
            # print('initial_l_foot:', initial_l_foot)
            # print('initial_r_foot:', initial_r_foot)
            print(model.getCOMVelocity())
            # solve problem with given constraints
            opt_values = solver.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)
            # interpolate at right frequency

            com_traj, l_foot_traj = solver.interpolateStep(opt_values, freq)

            # PLOTTING =============================================
            plt.figure()
            plt.clf()
            #
            # ######## COM ######
            plt.plot(opt_values['p'][0], opt_values['p'][1])

            # ######## ZMP ######
            zmp = list(np.array(opt_values['p']) - np.array(opt_values['a']) * (height_com / solver.grav))
            plt.plot(zmp[0], zmp[1], '-')

            alphal = np.sum(opt_values['alpha_l'], 0)
            alphar = np.sum(opt_values['alpha_r'], 0)
            ######## LEFT foot ######
            k = 0
            for pos_x, pos_y in zip(opt_values['l'][0], opt_values['l'][1]):
                if alphal[k] > 1e-6:
                    coord = solver.getEdges([pos_x, pos_y])
                    coord = cs.vertcat(coord, coord[0, :])
                    plt.plot(coord[:, 0], coord[:, 1], color='b')
                    plt.scatter(pos_x, pos_y)
                k += 1

            ######## RIGHT foot ######
            k = 0
            for pos_x, pos_y in zip(opt_values['r'][0], opt_values['r'][1]):
                if alphar[k].any() > 1e-6:
                    coord = solver.getEdges([pos_x, pos_y])
                    coord = cs.vertcat(coord, coord[0, :])
                    plt.plot(coord[:, 0], coord[:, 1], color='r')
                    plt.scatter(pos_x, pos_y)
                k += 1

            plt.show()
            exit()


            n_solve +=1
            flag_step = True
            t_start_step = t

        if flag_step:

            # set l_foot trajectory
            lfoot.translation[0] = l_foot_traj['x'][i]
            lfoot.translation[1] = l_foot_traj['y'][i]
            lfoot.translation[2] = l_foot_initial.translation[2] + l_foot_traj['z'][i]

            dlfoot[0] = l_foot_traj['dx'][i]
            dlfoot[1] = l_foot_traj['dy'][i]
            dlfoot[2] = l_foot_traj['dz'][i]

            # set com trajectory
            com.translation[0] = com_traj['x'][i]
            com.translation[1] = com_traj['y'][i]

            dcom[0] = com_traj['dx'][i]
            dcom[1] = com_traj['dy'][i]

            com_task.setPoseReference(com)
            com_task.setVelocityReference(dcom)

            ctrl_tasks[0].setPoseReference(lfoot)
            ctrl_tasks[0].setVelocityReference(dlfoot)

            i += 1
            if i >= len(l_foot_traj['x']):
                flag_step = False

        ci_solver.update(t, sim=True)

        t += 1./freq

        rate.sleep()

    #
    #
    #
    #
    #
    #
