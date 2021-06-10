# # /usr/bin/env python2.7
#
# from casadi import *
# from numpy import *
# import matplotlib
# import matplotlib.pyplot as plt
# import pprint
# import time
# from ci_solver import CartesianInterfaceSolver
#
# class Stepper:
#
#     def __init__(self, initial_ds_t, ss_1_t, ds_1_t, ss_2_t, final_ds_t):
#
#         # D L D R D // two steps
#         self.initial_ds_t = initial_ds_t
#         self.ss_1_t = ss_1_t
#         self.ds_1_t = ds_1_t
#         self.ss_2_t = ss_2_t
#         self.final_ds_t = final_ds_t
#
#         self.n_duration = 0.02  # single node duration
#         self.T = self.initial_ds_t + self.ss_1_t + self.ds_1_t + self.ss_2_t + self.final_ds_t  # 1.0  # time horizon
#         self.N = int(self.T / self.n_duration)  # number of control intervals
#
#         print 'duration of initial_ds_t:', self.initial_ds_t
#         print 'duration of ss_1_t:', self.ss_1_t
#         print 'duration of ds_1_t:', self.ds_1_t
#         print 'duration of ss_2_t:', self.ss_2_t
#         print 'duration of final_ds_t:', self.final_ds_t
#         print 'T:', self.T
#         print 'N:', self.N
#         print 'duration of a single node:', self.n_duration
#
#         self.initial_ds_n = int(self.initial_ds_t / self.n_duration)
#         self.ss_1_n = int(self.ss_1_t / self.n_duration)
#         self.ds_1_n = int(self.ds_1_t / self.n_duration)
#         self.ss_2_n = int(self.ss_2_t / self.n_duration)
#         self.final_ds_n = int(self.final_ds_t / self.n_duration)
#
#         print 'duration (in nodes) of initial ds:', self.initial_ds_n
#         print 'duration (in nodes) of first ss:', self.ss_1_n
#         print 'duration (in nodes) of middle ds:', self.ds_1_n
#         print 'duration (in nodes) of second ss:', self.ss_2_n
#         print 'duration (in nodes) of final ds:', self.final_ds_n
#
#         self.ds_1 = self.initial_ds_n
#         self.ss_1 = self.ds_1 + self.ss_1_n
#         self.ds_2 = self.ss_1 + self.ds_1_n
#         self.ss_2 = self.ds_2 + self.ss_2_n
#         self.ds_3 = self.ss_2 + self.final_ds_n
#
#         self.width_foot = 0.1
#         self.length_foot = 0.2
#
#         self.max_stride_x = 0.4
#         self.max_stride_y = self.width_foot / 2. + 0.5
#         self.min_stride_y = self.width_foot / 2. + 0.15
#
#         self.grav = 9.81
#
#         self.sym_c = SX
#
#         # state variables
#         self.p = self.sym_c.sym('p', 2)  # com position
#         self.v = self.sym_c.sym('v', 2)  # com velocity
#         self.a = self.sym_c.sym('a', 2)  # com acceleration
#
#         self.l = self.sym_c.sym('l', 2)  # left foot
#         self.r = self.sym_c.sym('r', 2)  # right foot
#
#         self.alpha_l = self.sym_c.sym('alpha_l', 4)  # left foot weigths [lu, ru, rh, lh]
#         self.alpha_r = self.sym_c.sym('alpha_r', 4)  # right foot weigths [lu, ru, rh, lh]
#
#         self.x = vertcat(self.p, self.v, self.a)  # , l, r, alpha_l, alpha_r) # state
#
#         # control variables
#         self.j = self.sym_c.sym('j', 2)  # com jerk
#         self.u = self.j  # control
#
#         # model equation
#         self.xdot = vertcat(self.v, self.a, self.j)
#
#         # Objective terms
#         L = sumsqr(self.u)
#         # Formulate discrete time dynamics
#         # Fixed step Runge-Kutta 4 integrator
#         M = 1  # RK4 steps per interval
#         self.dt = self.T / self.N / M
#         self.F = self.RK4(M, L, self.dt)
#
#         # augment state
#         self.z = vertcat(self.p, self.v, self.a, self.l, self.r, self.alpha_l, self.alpha_r)  # state
#
#     def RK4(self, M, L, dt):
#         """RK4 Runge-Kutta 4 integrator
#         Input:
#             L: objective function to integrate
#             M: RK steps
#             T: final time
#             N: number of shooting nodes:
#             x: state variables
#             u: controls
#         """
#
#         f = Function('f', [self.x, self.u], [self.xdot, L])
#         X0 = self.sym_c.sym('X0', self.x.size()[0])
#         U = self.sym_c.sym('U', self.u.size()[0])
#         X = X0
#         Q = 0
#
#         for j in range(M):
#             k1, k1_q = f(X, U)
#             k2, k2_q = f(X + dt / 2 * k1, U)
#             k3, k3_q = f(X + dt / 2 * k2, U)
#             k4, k4_q = f(X + dt * k3, U)
#             X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
#             Q = Q + dt / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
#
#         return Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])
#
#     def casadi_sum(self, x, axis=None, out=None):
#         assert out is None
#         if axis == 0:
#             return sum1(x)
#         elif axis == 1:
#             return sum2(x)
#         elif axis is None:
#             return sum1(sum2(x))
#         else:
#             raise Exception("Invalid argument for sum")
#
#     def getEdges(self, p):  # lu, ru, rh, lh
#
#         lu = DM([+ self.length_foot / 2., + self.width_foot / 2.])
#         ru = DM([+ self.length_foot / 2., - self.width_foot / 2.])
#         rh = DM([- self.length_foot / 2., - self.width_foot / 2.])
#         lh = DM([- self.length_foot / 2., + self.width_foot / 2.])
#
#         ft_vert = horzcat(p + lu, p + ru, p + rh, p + lh).T
#
#         return ft_vert
#
#     def addConstraint(self, Gk, LBGk, UBGk):
#         self.g += [Gk]
#         self.lbg += LBGk
#         self.ubg += UBGk
#
#     def addStance(self, zmp, wft_l_vert, wft_r_vert, k, stance, X):
#
#         if stance == 'L':
#             # swing leg L -->
#             # zmp must stay inside sole R
#             self.addConstraint(zmp - self.casadi_sum(wft_r_vert, 0).T, [0, 0], [0, 0])
#
#             # alpha (weights on vertices) must be 0 for the swing leg and 1 for the stance leg
#             self.addConstraint(self.casadi_sum(X[k][10:14], 0).T, [0], [0])
#             self.addConstraint(self.casadi_sum(X[k][14:18], 0).T, [1.], [1.])
#
#             # right foot must stay still
#             if k > 0:
#                 self.addConstraint(X[k][8:10] - X[k - 1][8:10], [0, 0], [0, 0])
#
#         elif stance == 'R':
#             # swing leg R -->
#             # zmp must stay inside sole L
#             self.addConstraint(zmp - self.casadi_sum(wft_l_vert, 0).T, [0, 0], [0, 0])
#
#             # alpha (weights on vertices) must be 0 for the swing leg and 1 for the stance leg
#             self.addConstraint(self.casadi_sum(X[k][10:14], 0).T, [1.], [1.])
#             self.addConstraint(self.casadi_sum(X[k][14:18], 0).T, [0.], [0.])
#
#             # left foot must stay still
#             if k > 0:
#                 self.addConstraint(X[k][6:8] - X[k - 1][6:8], [0, 0], [0, 0])
#
#         elif stance == 'D':
#             # double stance -->
#             # zmp must stay inside sole L and R
#             self.addConstraint(zmp - (self.casadi_sum(wft_l_vert, 0).T + self.casadi_sum(wft_r_vert, 0).T), [0, 0], [0, 0])
#
#             # alpha (weights on vertices) must be 1 for stance leg + swing leg
#             self.addConstraint(self.casadi_sum(X[k][10:18], 0), [1.], [1.])
#
#             # left foot AND right foot must stay still
#             if k > 0:
#                 self.addConstraint(X[k][6:8] - X[k - 1][6:8], [0, 0], [0, 0])
#                 self.addConstraint(X[k][8:10] - X[k - 1][8:10], [0, 0], [0, 0])
#
#     #######################################
#
#     def generateProblem(self, initial_com, heigth_com, l_foot, r_foot):
#
#         # Start with an empty NLP
#         w0 = []  # initial value for optimization variables
#         self.lbw = []  # lower bound on variables
#         self.ubw = []  # upper bound on variables
#         J = 0  # const function
#         self.g = []  # constraints
#         self.lbg = []  # lower constraint
#         self.ubg = []  # upper constraints
#
#         self.h = heigth_com
#         X = list()
#         U = list()
#
#         for k in range(self.N + 1):
#             # STATE
#             xk = self.sym_c.sym('X_' + str(k), self.z.size()[0])
#             X.append(xk)
#
#             if k == 0:  # at the beginning, position, velocity and acceleration to ZERO
#                 self.lbw += [initial_com[0, 0], initial_com[0, 1],  # com pos
#                              initial_com[1, 0], initial_com[1, 1],  # com vel
#                              initial_com[2, 0], initial_com[2, 1],  # com acc
#                              l_foot[0], l_foot[1],  # left foot pos
#                              r_foot[0], r_foot[1],  # right foot pos
#                              0., 0., 0., 0.,  # alpha l
#                              0., 0., 0., 0.  # alpha r
#                              ]
#                 self.ubw += [initial_com[0, 0], initial_com[0, 1],
#                              initial_com[1, 0], initial_com[1, 1],
#                              initial_com[2, 0], initial_com[2, 1],
#                              l_foot[0], l_foot[1],  # left foot pos
#                              r_foot[0], r_foot[1],  # right foot pos
#                              1., 1., 1., 1.,  # alpha l
#                              1., 1., 1., 1.  # alpha r
#                              ]
#
#             elif k == self.N:  # final state
#                 self.lbw += [-inf, -inf,  # com pos
#                              0., 0.,  # com vel
#                              0., 0.,  # com acc
#                              -inf, -inf,  # left foot pos
#                              -inf, -inf,  # right foot pos
#                              0., 0., 0., 0.,
#                              0., 0., 0., 0.,
#                              ]
#                 self.ubw += [inf, inf,  # com pos
#                              0., 0.,  # com vel
#                              0., 0.,  # com acc
#                              inf, inf,  # left foot pos
#                              inf, inf,  # right foot pos
#                              1., 1., 1., 1.,
#                              1., 1., 1., 1.,
#                              ]
#             else:
#                 self.lbw += [-inf, -inf,  # com pos
#                              -inf, -inf,  # com vel
#                              -inf, -inf,  # com acc
#                              -inf, -inf,  # left foot pos
#                              -inf, -inf,  # right foot pos
#                              0., 0., 0., 0.,  # alpha l
#                              0., 0., 0., 0.  # alpha r
#                              ]
#                 self.ubw += [inf, inf,  # com pos
#                              inf, inf,  # com vel
#                              inf, inf,  # com acc
#                              inf, inf,  # left foot pos
#                              inf, inf,  # right foot pos
#                              1., 1., 1., 1.,  # alpha l
#                              1., 1., 1., 1.  # alpha r
#                              ]
#
#             # initial guess for state
#             w0 += list(np.zeros(self.z.size()[0]))
#
#         # CONTROL (last loop does not have u)
#             if k < self.N:
#                 uk = self.sym_c.sym('U_' + str(k), self.u.size()[0])
#                 U.append(uk)
#
#                 self.lbw += [-1000., -1000.]
#                 self.ubw += [1000., 1000.]
#
#                 # minimize input
#                 J = J + 0.001 * sumsqr(U[k])
#
#                 # initial guess for control
#                 w0 += list(np.zeros(self.u.size()[0]))
#
#             if k > 0:
#                 ## forward integration
#                 Fk = self.F(x0=X[k - 1][0:6], p=U[k - 1])
#                 # Multiple Shooting (the result of the integrator [XInt[k-1]] must be the equal to the value of the next node)
#                 self.addConstraint(Fk['xf'] - X[k][0:6], list(np.zeros(self.x.size()[0])), list(np.zeros(self.x.size()[0])))
#
#             ## minimize velocity
#             J = J + sumsqr(X[k][2:4])
#
#             ## Regularization of alphas
#             Ak = X[k][10:18]
#             J = J + sumsqr(Ak)
#
#             ## WALKING SCHEDULER
#             ZMP = X[k][0:2] - X[k][4:6] * (self.h / self.grav)
#             Lk = X[k][6:8]
#             Rk = X[k][8:10]
#
#             ## add stride constraint
#             self.addConstraint(Lk - Rk, [-self.max_stride_x, -self.max_stride_y], [self.max_stride_x, self.max_stride_y])
#
#             ## minimize movement on y
#             J = J + 1000. * sumsqr((Lk[1] - Rk[1]) - self.min_stride_y)
#
#             # get weighted edges
#             wft_l_vert = X[k][10:14] * self.getEdges(Lk)  # lu, ru, rh, lh
#             wft_r_vert = X[k][14:18] * self.getEdges(Rk)  # lu, ru, rh, lh
#
#             if k <= self.ds_1:  # add double stance
#                 self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)
#             elif self.ds_1 < k <= self.ss_1:  # add single stance
#                 self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'L', X)
#             elif self.ss_1 < k <= self.ds_2:  # add double stance
#                 self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)
#             elif self.ds_2 < k <= self.ss_2:
#                 self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'R', X)
#             else:
#                 self.addStance(ZMP, wft_l_vert, wft_r_vert, k, 'D', X)
#
#
#         w = [None] * (len(X) + len(U))
#         w[0::2] = X
#         w[1::2] = U
#
#         prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*self.g)}
#         constraints = dict(lbw=self.lbw, ubw=self.ubw, lbg=self.lbg, ubg=self.ubg)
#
#         return prob, w0, constraints
#
#     def solve(self, prob, initial_guess, constraints):
#
#         print('================')
#         print('w:', prob['x'].shape)
#         print('lbw:', len(self.lbw))
#         print('ubw:', len(self.ubw))
#         print('g:', prob['g'].shape)
#         print('lbg:', len(self.lbg))
#         print('ubg:', len(self.ubg))
#         print('================')
#
#         # Create an NLP solver
#         solver = nlpsol('solver', 'ipopt', prob,
#                         {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 3, 'sb': 'yes'},
#                          'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3
#
#         # Solve the NLP
#
#         t = time.time()
#         sol = solver(x0=initial_guess, lbx=constraints['lbw'], ubx=constraints['ubw'], lbg=constraints['lbg'],
#                      ubg=constraints['ubg'])
#         print('duration:', time.time() - t)
#
#         w_opt = sol['x'].full().flatten()
#         return w_opt
#
#     def get_relevant_variables(self, w_opt):
#
#         feet_states = list()
#
#         relevant_nodes = [0, self.ss_2 + 1]  # self.ds_1_n+1,
#
#         num_var = self.z.size()[0] + self.u.size()[0]
#         for k in relevant_nodes:
#             sol_k = w_opt[num_var * k:(num_var * k + num_var)]
#             feet_states.append(dict(l=sol_k[6:8], r=sol_k[8:10]))
#
#         com_states = list()
#         for k in range(self.N):
#             sol_k = w_opt[num_var * k:(num_var * k + num_var)]
#             com_states.append(dict(p=list(sol_k[0:2]), v=list(sol_k[2:4]), a=list(sol_k[4:6]), j=sol_k[18:20]))
#
#         return com_states, feet_states
#
#     def plot(self, w_opt):  # Plot the solution
#         p_opt = []
#         v_opt = []
#         a_opt = []
#         j_opt = []
#         l_opt = []
#         r_opt = []
#         alpha_l_opt = []
#         alpha_r_opt = []
#
#         num_var = self.z.size()[0] + self.u.size()[0]
#         for k in range(self.N):
#             sol_k = w_opt[num_var * k:(num_var * k + num_var)]
#             p_opt += list(sol_k[0:2])
#             v_opt += list(sol_k[2:4])
#             a_opt += list(sol_k[4:6])
#             l_opt += list(sol_k[6:8])
#             r_opt += list(sol_k[8:10])
#             alpha_l_opt += list(sol_k[10:14])
#             alpha_r_opt += list(sol_k[14:18])
#             j_opt += list(sol_k[18:20])
#
#         sol_k = w_opt[-self.z.size()[0]:]
#         p_opt += list(sol_k[0:2])
#         v_opt += list(sol_k[2:4])
#         a_opt += list(sol_k[4:6])
#         l_opt += list(sol_k[6:8])
#         r_opt += list(sol_k[8:10])
#         alpha_l_opt += list(sol_k[10:14])
#         alpha_r_opt += list(sol_k[14:18])
#
#         tgrid = [self.T / self.N * k for k in range(self.N + 1)]
#         px = p_opt[0::2]
#         vx = v_opt[0::2]
#         ax = a_opt[0::2]
#         lx = l_opt[0::2]
#         rx = r_opt[0::2]
#         jx = j_opt[0::2]
#
#         zmpx = list(np.array(px) - np.array(ax) * (self.h / self.grav))
#
#         alphal = [sum(alpha_l_opt[x:x + 4]) for x in range(0, len(alpha_l_opt), 4)]
#         alphar = [sum(alpha_r_opt[x:x + 4]) for x in range(0, len(alpha_r_opt), 4)]
#
#         jx.insert(0, float('nan'))
#
#         ######## plot X #######################
#         plt.figure()
#         plt.clf()
#         plt.title('sagittal plane')
#         plt.plot(tgrid, px, '--')
#         plt.plot(tgrid, vx, '-')
#         plt.step(tgrid, ax, '-.')
#         plt.step(tgrid, jx, '-.')
#         plt.plot(tgrid, zmpx, '--')
#         plt.xlabel('t')
#         plt.legend(['px', 'vx', 'ax', 'ux', 'zmpx'])
#         plt.grid()
#
#         py = p_opt[1::2]
#         vy = v_opt[1::2]
#         ay = a_opt[1::2]
#         ly = l_opt[1::2]
#         ry = r_opt[1::2]
#         jy = j_opt[1::2]
#
#         zmpy = list(np.array(py) - np.array(ay) * (self.h / self.grav))
#         jy.insert(0, float('nan'))
#
#         ######## plot Y #######################
#         plt.figure()
#         plt.clf()
#         plt.title('lateral plane')
#         plt.plot(tgrid, py, '--')
#         plt.plot(tgrid, vy, '-')
#         plt.step(tgrid, ay, '-.')
#         plt.step(tgrid, jy, '-.')
#         plt.plot(tgrid, zmpy, '--')
#         plt.xlabel('t')
#         plt.legend(['py', 'vy', 'ay', 'uy', 'zmpy'])
#         plt.grid()
#
#         ######## plot x-y plane #######################
#
#         plt.figure()
#         plt.clf()
#         plt.title('zmp')
#         ##### com #####
#         plt.plot(px, py, '-')
#         ##### zmp #####
#         plt.plot(zmpx, zmpy, '-')
#         ######## LEFT foot ######
#         k = 0
#         for pos_x, pos_y in zip(lx, ly):
#             if alphal[k] > 1e-6:
#                 coord = self.getEdges([pos_x, pos_y])
#                 coord = vertcat(coord, coord[0, :])
#                 plt.plot(coord[:, 0], coord[:, 1], color='b')
#                 plt.scatter(pos_x, pos_y)
#             k += 1
#
#         ######## RIGHT foot ######
#         k = 0
#         for pos_x, pos_y in zip(rx, ry):
#             if alphar[k] > 1e-6:
#                 coord = self.getEdges([pos_x, pos_y])
#                 coord = vertcat(coord, coord[0, :])
#                 plt.plot(coord[:, 0], coord[:, 1], color='r')
#                 plt.scatter(pos_x, pos_y)
#             k += 1
#
#         plt.legend(['p', 'zmp'])
#         plt.xlabel('x')
#         plt.ylabel('y')
#
#         ############ Plot alpha ############
#         plt.figure()
#         plt.clf()
#         plt.title('alpha  sum')
#         plt.scatter(tgrid, alphal)
#         plt.scatter(tgrid, alphar, color='r')
#         plt.legend(['l', 'r'])
#         plt.xlabel('t')
#         plt.ylabel('weigth')
#
#         plt.show()
#
#     def interpolator(self, step_i, step_f, step_height, time, t_i, t_f, freq):
#
#         traj = dict()
#         traj_tot = float(freq) * float(time)
#         traj_len = float(freq) * float(t_f - t_i)
#         traj_len_before = float(freq) * float(t_i)
#         traj_len_after = float(freq) * float(time-t_f)
#         dt = 1. / float(freq)
#
#         traj['x'] = np.full(traj_len_before, step_i[0])
#         traj['y'] = np.full(traj_len_before, step_i[1])
#         traj['z'] = np.full(traj_len_before, 0.)
#
#         traj['dx'] = np.full(traj_len_before, 0.)
#         traj['dy'] = np.full(traj_len_before, 0.)
#         traj['dz'] = np.full(traj_len_before, 0.)
#
#         traj['ddx'] = np.full(traj_len_before, 0.)
#         traj['ddy'] = np.full(traj_len_before, 0.)
#         traj['ddz'] = np.full(traj_len_before, 0.)
#
#         t = np.linspace(0, 1, ceil(traj_len))
#
#         traj['x'] = np.append(traj['x'], (step_i[0] + (((6 * t - 15) * t + 10) * t ** 3) * (step_f[0] - step_i[0])))  # on the x
#         traj['y'] = np.append(traj['y'], (step_i[1] + (((6 * t - 15) * t + 10) * t ** 3) * (step_f[1] - step_i[1]))) # on the y
#         traj['z'] = np.append(traj['z'], (64 * t ** 3. * (1 - t) ** 3) * step_height) # on the z
#
#
#         traj['x'] = np.append(traj['x'], np.full(traj_len_after, step_f[0]))
#         traj['y'] = np.append(traj['y'], np.full(traj_len_after, step_f[1]))
#         traj['z'] = np.append(traj['z'], np.full(traj_len_after, 0.))
#
#
#         # compute velocity
#         traj['dx'] = np.append(traj['dx'], 30 * (t - 1) ** 2 * t ** 2 * (step_f[0] - step_i[0]))
#         traj['dy'] = np.append(traj['dy'], 30 * (t - 1) ** 2 * t ** 2 * (step_f[1] - step_i[1]))
#         traj['dz'] = np.append(traj['dz'], step_height * (t - 1) ** 2 * (t ** 2.0 * (192.0 - 192.0 * t) - 192 * t ** 3.0))
#
#         traj['dx'] = np.append(traj['dx'], np.full(traj_len_after, 0.))
#         traj['dy'] = np.append(traj['dy'], np.full(traj_len_after, 0.))
#         traj['dz'] = np.append(traj['dz'], np.full(traj_len_after, 0.))
#
#         # computeration
#         traj['ddx'] = np.append(traj['ddx'], 60 * t * (2 * t ** 2 - 3 * t + 1) * (step_f[0] - step_i[0]))
#         traj['ddy'] = np.append(traj['ddy'], 60 * t * (2 * t ** 2 - 3 * t + 1) * (step_f[1] - step_i[1]))
#         traj['ddz'] = np.append(traj['ddz'], step_height * (384.0 * t ** 1.0 - 2304.0 * t ** 2.0 + 3840.0 * t ** 3.0 - 1920.0 * t ** 4.0))
#
#         traj['ddx'] = np.append(traj['ddx'], np.full(traj_len_after, 0.))
#         traj['ddy'] = np.append(traj['ddy'], np.full(traj_len_after, 0.))
#         traj['ddz'] = np.append(traj['ddz'], np.full(traj_len_after, 0.))
#
#         # small hack for t filling
#         t = np.linspace(0, 1, len(traj['x']))
#
#         return t, traj
#
#
# if __name__ == '__main__':
#
#     np.set_printoptions(precision=3, suppress=True)
#
#     initial_ds = 0.2
#     ss_1 = 0.5
#     ds_1 = 0.
#     ss_2 = 0.
#     final_ds = 0.4
#     T_total = initial_ds + ss_1 + ds_1 + ss_2 + final_ds
#
#     stp = Stepper(initial_ds, ss_1, ds_1, ss_2, final_ds)
#
#     initial_l_foot = np.array([-0.128, 0.103, 0.])
#     initial_r_foot = np.array([-0.128, -0.103, 0.])
#     initial_com = np.array([[-0.067, 0.], [0.0, 0.], [0., 0.]])
#
#     # initial_l_foot = np.array([0., 0.1, 0.])
#     # initial_r_foot = np.array([0., -0.1, 0.])
#     # initial_com = np.array([[0., 0.], [0.4, 0.], [0., 0]])
#     height_com = 0.9
#     problem, initial_guess, constraints = stp.generateProblem(initial_com=initial_com, heigth_com=height_com, l_foot=initial_l_foot,
#                                                               r_foot=initial_r_foot)
#
#
#
#     # problem with zero initial velocity
#     w_opt = stp.solve(problem, initial_guess, constraints)
#
#     # problem with given velocity
#     # constraints['lbw'][2] = 0.5  # change velocity constraint
#     # constraints['ubw'][2] = 0.5  # change velocity constraint
#
#     # for i in range(10):
#     # w_opt = stp.solve(problem, w_opt, constraints)
#
#     com_states, feet_states = stp.get_relevant_variables(w_opt)
#     #
#     stp.plot(w_opt)
#
#     ## show step and com trajectories
#     # for i in range(len(feet_states)):
#     #     plt.scatter(feet_states[i]['l'][0], feet_states[i]['l'][1])
#     #     plt.scatter(feet_states[i]['r'][0], feet_states[i]['r'][1])
#     #
#     # plt.show()
#     #
#     # plt.plot([i['p'][0] for i in com_states], [i['p'][1] for i in com_states])
#     #
#     ## get the initial and the final position, get step height and interpolate
#     t, traj = stp.interpolator(step_i=feet_states[0]['l'], step_f=feet_states[1]['l'], step_height=0.5, time=T_total, t_i=initial_ds, t_f=initial_ds+ss_1, freq=100)
#
#     # position
#     # plt.figure()
#     # plt.plot(t, traj['x'])
#     # plt.plot(t, traj['y'])
#     # plt.plot(t, traj['z'])
#     # plt.legend(['x', 'y', 'z'])
#     # plt.xlabel('t')
#
#     # velocity
#     # plt.figure()
#     # plt.plot(t[1:], traj['dx'])
#     # plt.plot(t[1:], traj['dy'])
#     # plt.plot(t[1:], traj['dz'])
#     # plt.legend(['dx', 'dy', 'dz'])
#     # plt.xlabel('t')
#     #
#     # acceleration
#     # plt.figure()
#     # plt.plot(t[2:], traj['ddx'])
#     # plt.plot(t[2:], traj['ddy'])
#     # plt.plot(t[2:], traj['ddz'])
#     # plt.legend(['ddx', 'ddy', 'ddz'])
#
#     # plt.show()
#     plt.figure()
#     plt.plot(t, traj['x'])
#     plt.plot(t, traj['dx'])
#     plt.plot(t, traj['ddx'])
#     plt.legend(['x', 'dx', 'ddx'])
#     plt.show()
#
#     exit()
#     #################################################################################################
#     # if I use the same integrator over the states it should give the same result as the com found by the optimal controller
#     x_state = DM(initial_com.flatten())
#     for iter_u in range(len(com_states)):
#         Fk_2 = stp.F(x0=x_state[:, -1], p=com_states[iter_u]['j'])
#         x_state = horzcat(x_state, Fk_2['xf'])
#
#
#     plt.plot(x_state[0, :].full().flatten(), x_state[1, :].full().flatten(), 'r', linewidth=2)
#
#     #################################################################################################
#     # integrate 'multiplier' times for every loop
#     # basically change the resolution of the trajectory
#
#     state_1 = DM(initial_com.flatten())
#     #
#     multiplier = 2
#     dt_int = stp.T / stp.N / 1 / multiplier
#     integrator = stp.RK4(1, 1, dt_int)
#     #
#     for i in range(len(com_states)):
#         for multi_i in range(multiplier):
#             Fk_1 = integrator(x0=state_1[:, -1], p=com_states[i]['j'])
#             state_1 = horzcat(state_1, Fk_1['xf'])
#
#     print('state_1.shape', state_1.shape)
#     plt.plot(state_1[0, :].full().flatten(), state_1[1, :].full().flatten(), 'k--', linewidth=2)
#     #
#     plt.show()
