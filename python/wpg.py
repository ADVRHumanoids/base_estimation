#/usr/bin/env python2.7

from casadi import *
from numpy import *

import matplotlib.pyplot as plt
import pprint

class Stepper:

    def __init__(self, initial_ds_t, ss_t, final_ds_t):

        self.initial_ds_t = initial_ds_t
        self.ss_t = ss_t
        self.final_ds_t = final_ds_t

        self.n_duration = 0.01 # single node duration
        self.T = self.initial_ds_t + self.ss_t + self.final_ds_t  #1.0  # time horizon
        self.N = int(self.T / self.n_duration)  # number of control intervals



        print 'initial_ds_t:', self.initial_ds_t
        print 'ss_t:', self.ss_t
        print 'final_ds_t:', self.final_ds_t
        print 'n_duration:', self.n_duration
        print 'T:', self.T
        print 'N:', self.N

        print 'node duration of initial ds:', int(self.initial_ds_t/self.n_duration)
        print 'node duration of ss:', int(self.ss_t/self.n_duration)
        print 'node duration of final ds:', int(self.final_ds_t/self.n_duration)

        self.node_ss_begin = int(self.initial_ds_t/self.n_duration)
        self.node_ss_end = int(self.initial_ds_t/self.n_duration) + int(self.ss_t/self.n_duration)

        print 'node_ss_begin:', self.node_ss_begin
        print 'node_ss_end:', self.node_ss_end

        self.width_foot = 0.1
        self.length_foot = 0.2

        self.max_stride_x = 0.8
        self.max_stride_y = self.width_foot / 2. + 0.5
        self.min_stride_y = self.width_foot / 2. + 0.15

        self.grav = 9.81
        self.h = 1.

        self.sym_c = MX

        # state variables
        self.p = self.sym_c.sym('p', 2)  # com position
        self.v = self.sym_c.sym('v', 2)  # com velocity
        self.a = self.sym_c.sym('a', 2)  # com acceleration

        self.l = self.sym_c.sym('l', 2)  # left foot
        self.r = self.sym_c.sym('r', 2)  # right foot

        self.alpha_l = self.sym_c.sym('alpha_l', 4)  # left foot weigths [lu, ru, rh, lh]
        self.alpha_r = self.sym_c.sym('alpha_r', 4)  # right foot weigths [lu, ru, rh, lh]

        self.x = vertcat(self.p, self.v, self.a)  # , l, r, alpha_l, alpha_r) # state

        # control variables
        self.j = self.sym_c.sym('j', 2)  # com jerk
        self.u = self.j  # control

        # model equation
        self.xdot = vertcat(self.v, self.a, self.j)

        # Objective terms
        L = sumsqr(self.u)
        # Formulate discrete time dynamics
        # Fixed step Runge-Kutta 4 integrator
        M = 1  # RK4 steps per interval
        self.dt = self.T / self.N / M
        self.F = self.RK4(M, L, self.dt)

        # augment state
        self.z = vertcat(self.p, self.v, self.a, self.l, self.r, self.alpha_l, self.alpha_r)  # state

        # Start with an empty NLP
        self.w = []  # optimization variables along all the knots
        self.w0 = []  # initial value for optimization variables
        self.lbw = []  # lower bound on variables
        self.ubw = []  # upper bound on variables
        self.J = 0  # const function
        self.g = []  # constraints
        self.lbg = []  # lower constraint
        self.ubg = []  # upper constraints

        self.XInt = []

    def RK4(self, M, L, dt):
        """RK4 Runge-Kutta 4 integrator
        Input:
            L: objective function to integrate
            M: RK steps
            T: final time
            N: number of shooting nodes:
            x: state variables
            u: controls
        """

        f = Function('f', [self.x, self.u], [self.xdot, L])
        X0 = self.sym_c.sym('X0', self.x.size()[0])
        U = self.sym_c.sym('U', self.u.size()[0])
        X = X0
        Q = 0

        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + dt / 2 * k1, U)
            k3, k3_q = f(X + dt / 2 * k2, U)
            k4, k4_q = f(X + dt * k3, U)
            X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            Q = Q + dt / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

        return Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    def casadi_sum(self, x, axis=None, out=None):
        assert out is None
        if axis == 0:
            return sum1(x)
        elif axis == 1:
            return sum2(x)
        elif axis is None:
            return sum1(sum2(x))
        else:
            raise Exception("Invalid argument for sum")

    def getEdges(self, p): # lu, ru, rh, lh

        lu = DM([+ self.length_foot / 2., + self.width_foot / 2.])
        ru = DM([+ self.length_foot / 2., - self.width_foot / 2.])
        rh = DM([- self.length_foot / 2., - self.width_foot / 2.])
        lh = DM([- self.length_foot / 2., + self.width_foot / 2.])

        ft_vert = horzcat(p + lu, p + ru, p + rh, p + lh).T

        return ft_vert

    def addConstraint(self, Gk, LBGk, UBGk):
        self.g += [Gk]
        self.lbg += LBGk
        self.ubg += UBGk

    def addStance(self, zmp, wft_l_vert, wft_r_vert, k, stance, X):

        if stance == 'L':
            # swing leg L -->
            # zmp must stay inside sole R
            self.addConstraint(zmp - self.casadi_sum(wft_r_vert, 0).T, [0, 0], [0, 0])

            # alpha (weights on vertices) must be 0 for the swing leg and 1 for the stance leg
            self.addConstraint(self.casadi_sum(X[k][10:14], 0).T, [0], [0])
            self.addConstraint(self.casadi_sum(X[k][14:18], 0).T, [1.], [1.])

            # right foot must stay still
            if k > 0:
                self.addConstraint(X[k][8:10] - X[k-1][8:10], [0, 0], [0, 0])

        elif stance == 'R':
            # swing leg R -->
            # zmp must stay inside sole L
            self.addConstraint(zmp - self.casadi_sum(wft_l_vert, 0).T, [0, 0], [0, 0])

            # alpha (weights on vertices) must be 0 for the swing leg and 1 for the stance leg
            self.addConstraint(self.casadi_sum(X[k][10:14], 0).T, [1.], [1.])
            self.addConstraint(self.casadi_sum(X[k][14:18], 0).T, [0.], [0.])

            # left foot must stay still
            if k > 0:
                self.addConstraint(X[k][6:8] - X[k-1][6:8], [0, 0], [0, 0])

        elif stance == 'D':
            # double stance -->
            # zmp must stay inside sole L and R
            self.addConstraint(zmp - (self.casadi_sum(wft_l_vert, 0).T + self.casadi_sum(wft_r_vert, 0).T), [0, 0], [0, 0])
            # alpha (weights on vertices) must be 1 for stance leg + swing leg
            self.addConstraint(self.casadi_sum(X[k][10:18], 0), [1.], [1.])

            # left foot AND right foot must stay still
            if k > 0:
                self.addConstraint(X[k][6:8] - X[k-1][6:8], [0, 0], [0, 0])
                self.addConstraint(X[k][8:10] - X[k-1][8:10], [0, 0], [0, 0])

#######################################

    def generateProblem(self, initial_com, l_foot, r_foot):

        X = list()
        U = list()

        for k in range(self.N+1):
            # STATE
            xk = self.sym_c.sym('X_' + str(k), self.z.size()[0])
            X.append(xk)

            if k == 0:  # at the beginning, position, velocity and acceleration to ZERO
                self.lbw += [initial_com[0, 0], initial_com[0, 1],  # com pos
                             initial_com[1, 0], initial_com[1, 1],  # com vel
                             initial_com[2, 0], initial_com[2, 1],  # com acc
                             l_foot[0], l_foot[1],  # left foot pos
                             r_foot[0], r_foot[1],  # right foot pos
                             0., 0., 0., 0.,  # alpha l
                             0., 0., 0., 0.  # alpha r
                            ]
                self.ubw += [initial_com[0, 0], initial_com[0, 1],
                             initial_com[1, 0], initial_com[1, 1],
                             initial_com[2, 0], initial_com[2, 1],
                             l_foot[0], l_foot[1],  # left foot pos
                             r_foot[0], r_foot[1],  # right foot pos
                             1., 1., 1., 1.,  # alpha l
                             1., 1., 1., 1.  # alpha r
                            ]

            elif k == self.N: # final state
                self.lbw += [-100., -100.,  # com pos
                            0., 0.,  # com vel
                            0., 0.,  # com acc
                            -100., -100.,  # left foot pos
                            -100., -100.,  # right foot pos
                            0., 0., 0., 0.,
                            0., 0., 0., 0.,
                            ]
                self.ubw += [100., 100.,  # com pos
                            0., 0.,  # com vel
                            0., 0.,  # com acc
                            100., 100.,  # left foot pos
                            100., 100.,  # right foot pos
                            1., 1., 1., 1.,
                            1., 1., 1., 1.,
                            ]
            else:
                self.lbw += [-100., -100., # com pos
                            -100., -100., # com vel
                            -100., -100., # com acc
                            -100., -100., # left foot pos
                            -100., -100., # right foot pos
                            0., 0., 0., 0.,  # alpha l
                            0., 0., 0., 0.  # alpha r
                            ]
                self.ubw += [100., 100., # com pos
                            100., 100., # com vel
                            100., 100., # com acc
                            100., 100., # left foot pos
                            100., 100., # right foot pos
                            1., 1., 1., 1.,  # alpha l
                            1., 1., 1., 1.  # alpha r
                            ]

            # initial guess for state
            self.w0 += list(np.zeros(self.z.size()[0]))

            # CONTROL (last loop does not have u)
            if k < self.N:
                uk = self.sym_c.sym('U_' + str(k), self.u.size()[0])
                U.append(uk)

                # minimize input
                self.J = self.J + sumsqr(U[k])

                self.lbw += [-1000., -1000.]
                self.ubw += [1000., 1000.]

                # initial guess for control
                self.w0 += list(np.zeros(self.u.size()[0]))

            if k > 0:
                # forward integration
                Fk = self.F(x0=X[k-1][0:6], p=U[k-1])
                # Multiple Shooting (the result of the integrator [XInt[k-1]] must be the equal to the value of the next node)
                self.addConstraint(Fk['xf'] - X[k][0:6], list(np.zeros(self.x.size()[0])), list(np.zeros(self.x.size()[0])))

            self.J = self.J + sumsqr(X[k][0:6])

            # Regularization of alphas
            Ak = X[k][10:18]
            self.J = self.J + sumsqr(Ak)

            #WALKING SCHEDULER
            ZMP = X[k][0:2] - X[k][4:6] * (self.h / self.grav)
            Lk = X[k][6:8]
            Rk = X[k][8:10]

            self.addConstraint(Lk - Rk, [-self.max_stride_x, -self.max_stride_y], [self.max_stride_x, self.max_stride_y])

            self.J = self.J + 1000.*sumsqr((Lk[1] - Rk[1]) - self.min_stride_y)

            # get weighted edges
            wft_l_vert = X[k][10:14] * self.getEdges(Lk)  # lu, ru, rh, lh
            wft_r_vert = X[k][14:18] * self.getEdges(Rk)  # lu, ru, rh, lh

            initial_ds_n = int(self.initial_ds_t/self.n_duration)
            ss_n = int(self.ss_t/self.n_duration)
            # final_ds_n = int(self.final_ds_t/self.n_duration)

            if k <= initial_ds_n:  # add INITIAL double stance
                self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)
            elif initial_ds_n < k < initial_ds_n + ss_n:  # add single stance
                self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'L', X)
            else:  # add double stance
                self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)

            # if k <= int(self.ds_duration/self.n_duration):  # initial double stance
            #     self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)
            # elif (k > int(self.ds_duration/self.n_duration) and k < int(self.ss_duration/self.n_duration)):
            #     self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'L', X)
            # elif (k > int(self.ss_duration/self.n_duration) and k < int(self.ds_duration/self.n_duration)):  # add double stance
            #     self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)


        w = [None]*(len(X)+len(U))
        w[0::2] = X
        w[1::2] = U

        prob = {'f': self.J, 'x': vertcat(*w), 'g': vertcat(*self.g)}

        return prob

    def solve(self, prob):
        # Create an NLP solver
        solver = nlpsol('solver', 'ipopt', prob)
        # Solve the NLP

        sol = solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg)
        w_opt = sol['x'].full().flatten()
        return w_opt

    def get_relevant_variables(self, w_opt):

        feet_states = list()

        relevant_nodes = [0, self.node_ss_end+1]

        num_var = self.z.size()[0] + self.u.size()[0]
        for k in relevant_nodes:
            sol_k = w_opt[num_var*k:(num_var*k+num_var)]
            feet_states.append(dict(l=sol_k[6:8], r=sol_k[8:10]))

        com_states = list()
        for k in range(self.N):
            sol_k = w_opt[num_var*k:(num_var*k+num_var)]
            com_states.append(dict(p=list(sol_k[0:2]), v=list(sol_k[2:4]), a=list(sol_k[4:6])))

        return com_states, feet_states

    def plot(self, w_opt):# Plot the solution
        p_opt = []
        v_opt = []
        a_opt = []
        j_opt = []
        l_opt = []
        r_opt = []
        alpha_l_opt = []
        alpha_r_opt = []

        num_var = self.z.size()[0] + self.u.size()[0]
        for k in range(self.N):
            sol_k = w_opt[num_var*k:(num_var*k+num_var)]
            p_opt += list(sol_k[0:2])
            v_opt += list(sol_k[2:4])
            a_opt += list(sol_k[4:6])
            l_opt += list(sol_k[6:8])
            r_opt += list(sol_k[8:10])
            alpha_l_opt += list(sol_k[10:14])
            alpha_r_opt += list(sol_k[14:18])
            j_opt += list(sol_k[18:20])

        sol_k = w_opt[-self.z.size()[0]:]
        p_opt += list(sol_k[0:2])
        v_opt += list(sol_k[2:4])
        a_opt += list(sol_k[4:6])
        l_opt += list(sol_k[6:8])
        r_opt += list(sol_k[8:10])
        alpha_l_opt += list(sol_k[10:14])
        alpha_r_opt += list(sol_k[14:18])

        tgrid = [self.T/self.N*k for k in range(self.N+1)]
        px = p_opt[0::2]
        vx = v_opt[0::2]
        ax = a_opt[0::2]
        lx = l_opt[0::2]
        rx = r_opt[0::2]
        jx = j_opt[0::2]

        zmpx = list(np.array(px) - np.array(ax)*(self.h/self.grav))

        alphal = [sum(alpha_l_opt[x:x+4]) for x in range(0, len(alpha_l_opt), 4)]
        alphar = [sum(alpha_r_opt[x:x+4]) for x in range(0, len(alpha_r_opt), 4)]

        jx.insert(0,float('nan'))

        ######## plot X #######################
        plt.figure()
        plt.clf()
        plt.title('sagittal plane')
        plt.plot(tgrid, px, '--')
        plt.plot(tgrid, vx, '-')
        plt.step(tgrid, ax, '-.')
        plt.step(tgrid, jx, '-.')
        plt.plot(tgrid, zmpx, '--')
        plt.xlabel('t')
        plt.legend(['px', 'vx', 'ax', 'ux', 'zmpx'])
        plt.grid()

        py = p_opt[1::2]
        vy = v_opt[1::2]
        ay = a_opt[1::2]
        ly = l_opt[1::2]
        ry = r_opt[1::2]
        jy = j_opt[1::2]

        zmpy = list(np.array(py) - np.array(ay)*(self.h/self.grav))
        jy.insert(0,float('nan'))

        ######## plot Y #######################
        plt.figure()
        plt.clf()
        plt.title('lateral plane')
        plt.plot(tgrid, py, '--')
        plt.plot(tgrid, vy, '-')
        plt.step(tgrid, ay, '-.')
        plt.step(tgrid, jy, '-.')
        plt.plot(tgrid, zmpy, '--')
        plt.xlabel('t')
        plt.legend(['py', 'vy', 'ay', 'uy', 'zmpy'])
        plt.grid()

        ######## plot x-y plane #######################

        plt.figure()
        plt.clf()
        plt.title('zmp')
        ##### com #####
        plt.plot(px, py, '-')
        ##### zmp #####
        plt.plot(zmpx, zmpy, '-')
        ######## LEFT foot ######
        k = 0
        for pos_x, pos_y in zip(lx, ly):
            if alphal[k] > 1e-6:
                coord = self.getEdges([pos_x, pos_y])
                coord = vertcat(coord, coord[0, :])
                plt.plot(coord[:, 0], coord[:, 1], color='b')
                plt.scatter(pos_x, pos_y)
            k += 1

        ######## RIGHT foot ######
        k = 0
        for pos_x, pos_y in zip(rx, ry):
            if alphar[k] > 1e-6:
                coord = self.getEdges([pos_x, pos_y])
                coord = vertcat(coord, coord[0, :])
                plt.plot(coord[:, 0], coord[:, 1], color='r')
                plt.scatter(pos_x, pos_y)
            k += 1


        plt.legend(['p','zmp'])
        plt.xlabel('x')
        plt.ylabel('y')

        ############ Plot alpha ############
        plt.figure()
        plt.clf()
        plt.title('alpha  sum')
        plt.scatter(tgrid, alphal)
        plt.scatter(tgrid, alphar, color='r')
        plt.legend(['l','r'])
        plt.xlabel('t')
        plt.ylabel('weigth')

        plt.show()

    def interpolator(self, step_i, step_f, step_height, time, freq):

        traj_len = float(freq) * float(time)
        t = np.linspace(0,1, traj_len)

        traj_x = step_i[0] + (((6*t - 15)*t + 10)* t**3) * (step_f[0] - step_i[0])  # on the x
        traj_y = step_i[1] + (((6*t - 15)*t + 10)* t**3) * (step_f[1] - step_i[1])  # on the y
        traj_z = (64 * t**3. * (1 - t)**3) * step_height # on the z
        return t, traj_x, traj_y, traj_z



if __name__ == '__main__':

    initial_ds = 0.5
    ss = 0.1
    final_ds = 0.5
    stp = Stepper(initial_ds, ss, final_ds)

    initial_l_foot = np.array([0., 0.1, 0.])
    initial_r_foot = np.array([0., -0.1, 0.])
    initial_com = np.array([[0., 0.], [0.5, 0.], [0., 0.]])

    problem = stp.generateProblem(initial_com=initial_com, l_foot=initial_l_foot, r_foot=initial_r_foot)

    w_opt = stp.solve(problem)

    com_states, feet_states = stp.get_relevant_variables(w_opt)

    # stp.plot(w_opt)
    for i in range(len(feet_states)):
        plt.scatter(feet_states[i]['l'][0], feet_states[i]['l'][1])
        plt.scatter(feet_states[i]['r'][0], feet_states[i]['r'][1])

    plt.plot([i['p'][0] for i in com_states], [i['p'][1] for i in com_states])

    t, traj_x, traj_y, traj_z = stp.interpolator(step_i=feet_states[0]['l'], step_f=feet_states[1]['l'], step_height=0.5, time=ss, freq=100)

    print len(t)
    plt.figure()
    plt.plot(t, traj_x)
    plt.plot(t, traj_y)
    plt.plot(t, traj_z)
    plt.legend(['x', 'y', 'z'])
    plt.xlabel('t')
    plt.show()



