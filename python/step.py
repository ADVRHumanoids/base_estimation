# /usr/bin/env python3

from horizon import Problem
from horizon import RK4
from horizon import casadi_sum
from horizon import interpolator

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

def plot(w_opt):  # Plot the solution
    p_opt = []
    v_opt = []
    a_opt = []
    j_opt = []
    l_opt = []
    r_opt = []
    alpha_l_opt = []
    alpha_r_opt = []

    num_var = 20
    for k in range(N):
        sol_k = w_opt[num_var * k:(num_var * k + num_var)]
        p_opt += list(sol_k[0:2])
        v_opt += list(sol_k[2:4])
        a_opt += list(sol_k[4:6])
        l_opt += list(sol_k[6:8])
        r_opt += list(sol_k[8:10])
        alpha_l_opt += list(sol_k[10:14])
        alpha_r_opt += list(sol_k[14:18])
        j_opt += list(sol_k[18:20])

    sol_k = w_opt[-18:]
    p_opt += list(sol_k[0:2])
    v_opt += list(sol_k[2:4])
    a_opt += list(sol_k[4:6])
    l_opt += list(sol_k[6:8])
    r_opt += list(sol_k[8:10])
    alpha_l_opt += list(sol_k[10:14])
    alpha_r_opt += list(sol_k[14:18])

    tgrid = [T / N * k for k in range(N + 1)]
    px = p_opt[0::2]
    vx = v_opt[0::2]
    ax = a_opt[0::2]
    lx = l_opt[0::2]
    rx = r_opt[0::2]
    jx = j_opt[0::2]

    zmpx = list(np.array(px) - np.array(ax) * (height_com / grav))

    alphal = [sum(alpha_l_opt[x:x + 4]) for x in range(0, len(alpha_l_opt), 4)]
    alphar = [sum(alpha_r_opt[x:x + 4]) for x in range(0, len(alpha_r_opt), 4)]

    print(alphal)
    print(alphar)

    jx.insert(0, float('nan'))

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

    zmpy = list(np.array(py) - np.array(ay) * (height_com / grav))
    jy.insert(0, float('nan'))

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
            coord = getEdges([pos_x, pos_y])
            coord = cs.vertcat(coord, coord[0, :])
            plt.plot(coord[:, 0], coord[:, 1], color='b')
            plt.scatter(pos_x, pos_y)
        k += 1

    ######## RIGHT foot ######
    k = 0
    for pos_x, pos_y in zip(rx, ry):
        if alphar[k] > 1e-6:
            coord = getEdges([pos_x, pos_y])
            coord = cs.vertcat(coord, coord[0, :])
            plt.plot(coord[:, 0], coord[:, 1], color='r')
            plt.scatter(pos_x, pos_y)
        k += 1

    plt.legend(['p', 'zmp'])
    plt.xlabel('x')
    plt.ylabel('y')

    ############ Plot alpha ############
    plt.figure()
    plt.clf()
    plt.title('alpha  sum')
    plt.scatter(tgrid, alphal)
    plt.scatter(tgrid, alphar, color='r')
    plt.legend(['l', 'r'])
    plt.xlabel('t')
    plt.ylabel('weigth')

    plt.show()


def getEdges(p):  # lu, ru, rh, lh

    lu = cs.DM([+ length_foot / 2., + width_foot / 2.])
    ru = cs.DM([+ length_foot / 2., - width_foot / 2.])
    rh = cs.DM([- length_foot / 2., - width_foot / 2.])
    lh = cs.DM([- length_foot / 2., + width_foot / 2.])

    ft_vert = cs.horzcat(p + lu, p + ru, p + rh, p + lh).T

    return ft_vert


np.set_printoptions(precision=3, suppress=True)

rospy.init_node('step')
roscpp_init('step', [])

# PREPARE ROBOT
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

robot.sense()
model.syncFrom(robot)
model.update()

# CREATE CARTESIAN SOLVER
freq = 100.
ctrl_points = {0: 'l_sole', 1: 'r_sole'}
ci_solver = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=1./freq, ctrl_points=ctrl_points)
print('Created cartesian interface.')

ctrl_tasks, com_task = ci_solver.getTasks()

l_foot_initial = Affine3()
l_foot_initial.translation = model.getPose(ctrl_tasks[0].getName()).translation
l_foot_initial.linear = model.getPose(ctrl_tasks[0].getName()).linear

r_foot_initial = Affine3()
r_foot_initial.translation = model.getPose(ctrl_tasks[1].getName()).translation
r_foot_initial.linear = model.getPose(ctrl_tasks[1].getName()).linear

com_initial = model.getCOM()


initial_ds_t = 0.2
ss_1_t = 0.5
ds_1_t = 0.
ss_2_t = 0.
final_ds_t = 0.4
T_total = initial_ds_t + ss_1_t + ds_1_t + ss_2_t + final_ds_t

n_duration = 0.02  # single node duration

T = initial_ds_t + ss_1_t + ds_1_t + ss_2_t + final_ds_t  # 1.0  # time horizon
N = int(T / n_duration)  # number of control intervals

initial_l_foot = np.array([l_foot_initial.translation[0], l_foot_initial.translation[1], 0.])
initial_r_foot = np.array([r_foot_initial.translation[0], r_foot_initial.translation[1], 0.])

print('initial_l_foot:', initial_l_foot)
print('initial_r_foot:', initial_r_foot)

height_com = model.getCOM()[2] - model.getPose(ctrl_tasks[0].getName()).translation[2]

initial_com = np.array([[model.getCOM()[0],  model.getCOM()[1]], [0.2, 0.], [0., 0.]])

print('initial_com:', initial_com)

prb = Problem(N)

print('duration of initial_ds_t: {}'.format(initial_ds_t))
print('duration of ss_1_t: {}'.format(ss_1_t))
print('duration of ds_1_t: {}'.format(ds_1_t))
print('duration of ss_2_t: {}'.format(ss_2_t))
print('duration of final_ds_t: {}'.format(final_ds_t))
print('T: {}'.format(T))
print('N: {}'.format(N))
print('duration of a single node: {}'.format(n_duration))

initial_ds_n = int(initial_ds_t / n_duration)
ss_1_n = int(ss_1_t / n_duration)
ds_1_n = int(ds_1_t / n_duration)
ss_2_n = int(ss_2_t / n_duration)
final_ds_n = int(final_ds_t / n_duration)

print('duration (in nodes) of initial ds: {}'.format(initial_ds_n))
print('duration (in nodes) of first ss: {}'.format(ss_1_n))
print('duration (in nodes) of middle ds: {}'.format(ds_1_n))
print('duration (in nodes) of second ss: {}'.format(ss_2_n))
print('duration (in nodes) of final ds: {}'.format(final_ds_n))

ds_1 = initial_ds_n
ss_1 = ds_1 + ss_1_n
ds_2 = ss_1 + ds_1_n
ss_2 = ds_2 + ss_2_n
ds_3 = ss_2 + final_ds_n

width_foot = 0.1
length_foot = 0.2

max_stride_x = 0.4
max_stride_y = width_foot / 2. + 0.5
min_stride_y = width_foot / 2. + 0.15

grav = 9.81

sym_c = cs.SX

# state variables
p = sym_c.sym('p', 2)  # com position
v = sym_c.sym('v', 2)  # com velocity
a = sym_c.sym('a', 2)  # com acceleration
x = cs.vertcat(p, v, a)  # , l, r, alpha_l, alpha_r) # state
# control variables
j = sym_c.sym('j', 2)  # com jerk
u = j  # control
# model equation
xdot = cs.vertcat(v, a, j)

# Objective terms
L = cs.sumsqr(u)
# Formulate discrete time dynamics
# Fixed step Runge-Kutta 4 integrator
M = 1  # RK4 steps per interval
dt = T / N / M
F = RK4(M, L, x, u, xdot, dt)

prb.setVariable('x', 6)
prb.setVariable('x-1', 6)

prb.setVariable('l', 2)
prb.setVariable('r', 2)

prb.setVariable('l-1', 2)
prb.setVariable('r-1', 2)

prb.setVariable('alpha_l', 4)
prb.setVariable('alpha_r', 4)

prb.setVariable('u', 2)
prb.setVariable('u-1', 2)
prb_vars = prb.getVariable()

# zmp variable
zmp = prb_vars['x'][0:2] - prb_vars['x'][4:6] * (height_com / grav)
prb.setFunction('zmp', zmp)

# integrator for multiple shooting
integrator = RK4(M, L, x, u, xdot, dt)
x_int = integrator(x0=prb_vars['x-1'], p=prb_vars['u-1'])

prb.setFunction('x_int', x_int)

wl_vert = prb_vars['alpha_l'] * getEdges(prb_vars['l'])
wr_vert = prb_vars['alpha_r'] * getEdges(prb_vars['r'])

prb.setFunction('wl_vert', wl_vert)
prb.setFunction('wr_vert', wr_vert)

prb_funs = prb.getFunction()

# todo remember that the last node is N+1! change something
# add constraints
prb.ct.setConstraintFunction('multiple_shooting', prb_funs['x_int']['xf'] - prb_vars['x'], nodes=[1, N+1], bounds=dict(ubg=[0., 0., 0., 0., 0., 0.], lbg=[0., 0., 0., 0., 0., 0.]))
prb.ct.setConstraintFunction('stride', prb_vars['l'] - prb_vars['r'], bounds=dict(lbg=[-max_stride_x, -max_stride_y], ubg=[max_stride_x, max_stride_y]))

# # double stance
prb.ct.setConstraintFunction('ds1_zmp_stab', prb_funs['zmp'] - (casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T), nodes=[0, initial_ds_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds1_contacts', casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T, nodes=[0, initial_ds_n+1], bounds=dict(lbg=[1.], ubg=[1.]))
prb.ct.setConstraintFunction('ds1_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[1, initial_ds_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds1_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[1, initial_ds_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
#
# #single
prb.ct.setConstraintFunction('ss1_zmp_stab', prb_funs['zmp'] - casadi_sum(prb_funs['wr_vert'], 0).T, nodes=[initial_ds_n+1, ds_1+ss_1_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ss1_contacts_l', casadi_sum(prb_vars['alpha_l'], 0).T, nodes=[initial_ds_n+1, ds_1+ss_1_n+1], bounds=dict(lbg=[0.], ubg=[0.]))
prb.ct.setConstraintFunction('ss1_contacts_r', casadi_sum(prb_vars['alpha_r'], 0).T, nodes=[initial_ds_n+1, ds_1+ss_1_n+1], bounds=dict(lbg=[1.], ubg=[1.]))
prb.ct.setConstraintFunction('ss1_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[initial_ds_n+1, ds_1+ss_1_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

# # double stance ds_2 + self.ss_2_n
prb.ct.setConstraintFunction('ds2_zmp_stab', prb_funs['zmp'] - (casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T), nodes=[ds_1+ss_1_n+1, ss_1 + ds_1_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds2_contacts', casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T, nodes=[ds_1+ss_1_n+1, ss_1 + ds_1_n+1], bounds=dict(lbg=[1.], ubg=[1.]))
prb.ct.setConstraintFunction('ds2_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[ds_1+ss_1_n+1, ss_1 + ds_1_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds2_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[ds_1+ss_1_n+1, ss_1 + ds_1_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
#
# #single stance
prb.ct.setConstraintFunction('ss2_zmp_stab', prb_funs['zmp'] - casadi_sum(prb_funs['wl_vert'], 0).T, nodes=[ss_1 + ds_1_n+1, ds_2 + ss_2_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ss2_contacts_l', casadi_sum(prb_vars['alpha_l'], 0).T, nodes=[ss_1 + ds_1_n+1, ds_2 + ss_2_n+1], bounds=dict(lbg=[1.], ubg=[1.]))
prb.ct.setConstraintFunction('ss2_contacts_r', casadi_sum(prb_vars['alpha_r'], 0).T, nodes=[ss_1 + ds_1_n+1, ds_2 + ss_2_n+1], bounds=dict(lbg=[0.], ubg=[0.]))
prb.ct.setConstraintFunction('ss2_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[ss_1 + ds_1_n+1, ds_2 + ss_2_n+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
#
# # double stance
prb.ct.setConstraintFunction('ds3_zmp_stab', prb_funs['zmp'] - (casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T), nodes=[ds_2 + ss_2_n+1, N+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds3_contacts', casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T, nodes=[ds_2 + ss_2_n+1, N+1], bounds=dict(lbg=[1.], ubg=[1.]))
prb.ct.setConstraintFunction('ds3_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[ds_2 + ss_2_n+1, N+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
prb.ct.setConstraintFunction('ds3_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[ds_2 + ss_2_n+1, N+1], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))


# add cost functions
prb.setCostFunction('minimize_input', 0.001 * cs.sumsqr(prb_vars['u']), nodes=[0, N])
prb.setCostFunction('minimize_velocity', cs.sumsqr(prb_vars['x'][2:4]))
prb.setCostFunction('minimize_alpha', cs.sumsqr(prb_vars['alpha_l']) + cs.sumsqr(prb_vars['alpha_r'])) # todo correct?
prb.setCostFunction('minimize_stride_y', 1000. * cs.sumsqr((prb_vars['l'][1] - prb_vars['r'][1]) - min_stride_y))


w, g = prb.buildProblem()


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

prb.setStateBoundsFromName('x', nodes=0, lbw=initial_lbw_com, ubw=initial_ubw_com)
#
prb.setStateBoundsFromName('l', nodes=0, lbw=[initial_l_foot[0], initial_l_foot[1]], ubw=[initial_l_foot[0], initial_l_foot[1]])
prb.setStateBoundsFromName('r', nodes=0, lbw=[initial_r_foot[0], initial_r_foot[1]], ubw=[initial_r_foot[0], initial_r_foot[1]])
#
prb.setStateBoundsFromName('x', nodes=N, lbw=final_lbw_com, ubw=final_ubw_com)
#
prb.setStateBoundsFromName('alpha_l', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])
prb.setStateBoundsFromName('alpha_r', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])

prb.setStateBoundsFromName('u', nodes=[0, N+1], lbw=[-1000., -1000.], ubw=[1000., 1000.])

w_opt = prb.solveProblem(w, g)

with open('your_file.txt', 'w') as f:
    for item in w_opt:
        f.write("%s\n" % item)

opt_values = prb.getOptimizedVariables(w_opt)

# plot(w_opt)


# plt.figure()
# plt.clf()
#
# ######## COM ######
# plt.plot(opt_values['p'][0], opt_values['p'][1])
#
# alphal = np.sum(opt_values['alpha_l'], 0)
# alphar = np.sum(opt_values['alpha_r'], 0)
#
#
# ######## LEFT foot ######
# k = 0
# for pos_x, pos_y in zip(opt_values['l'][0], opt_values['l'][1]):
#     if alphal[k] > 1e-6:
#         coord = getEdges([pos_x, pos_y])
#         coord = cs.vertcat(coord, coord[0, :])
#         plt.plot(coord[:, 0], coord[:, 1], color='b')
#         plt.scatter(pos_x, pos_y)
#     k += 1
#
# ######## RIGHT foot ######
# k = 0
# for pos_x, pos_y in zip(opt_values['r'][0], opt_values['r'][1]):
#     if alphar[k].any() > 1e-6:
#         coord = getEdges([pos_x, pos_y])
#         coord = cs.vertcat(coord, coord[0, :])
#         plt.plot(coord[:, 0], coord[:, 1], color='r')
#         plt.scatter(pos_x, pos_y)
#     k += 1
#
# #
# plt.show()


# = = = = = = = = = = = = = = = = = = = = = = = = = robot = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = =

# interpolate the CoM at a higher frequency (from 0.2 to 0.1)
com_interpol = cs.DM(initial_com.flatten())


multiplier = 2
dt = n_duration
dt_int = dt / 1 / multiplier

integrator = RK4(M, L, x, u, xdot, dt_int)

#interpolate CoM using initial_com as x_0 and the optimal input found before, but with a frequency multiplied
# apply *multiplier* time the input at every loop
for i in range(opt_values['u'].shape[1]):
    for multi_i in range(multiplier):
        Fk = integrator(x0=com_interpol[:, -1], p=opt_values['u'][:, i])
        com_interpol = cs.horzcat(com_interpol, Fk['xf'])

com_traj_x = com_interpol[0, :].full().flatten()
com_traj_y = com_interpol[1, :].full().flatten()


relevant_nodes = [0, ds_2 + ss_2_n + 1]
# "manual interpolator
t, l_foot_traj = interpolator(step_i=opt_values['l'][:,relevant_nodes[0]], step_f=opt_values['l'][:,relevant_nodes[1]], step_height=0.05, time=T_total, t_i =initial_ds_t, t_f=initial_ds_t + ss_1_t, freq=freq)


lfoot = Affine3()
lfoot.translation = l_foot_initial.translation
lfoot.linear = l_foot_initial.linear

dlfoot = Affine3()
#
#
com = Affine3()
com.translation = com_initial

print('number of commands for a T =', T_total, 'trajectory with f =', freq, ': ', int(T_total*freq))
if len(l_foot_traj['x']) == len(l_foot_traj['y']) == len(l_foot_traj['z']):
    print('duration of foot trajectory is:', ss_1, '. Trajectory vector of length:', len(l_foot_traj['x']))

if len(com_traj_x) == len(com_traj_y):
    print('duration of com trajectory is', T_total, '. Trajectory vector of length:', len(com_traj_x))

intial_t = 0
t = 0
i = 0
rate = rospy.Rate(freq)
#lambda

com_task.setActivationState(pyci.ActivationState.Enabled)
ctrl_tasks[0].setActivationState(pyci.ActivationState.Enabled)

i = 0
while t < (intial_t + T_total):

    # set l_foot trajectory
    lfoot.translation[0] = l_foot_traj['x'][i]
    lfoot.translation[1] = l_foot_traj['y'][i]
    lfoot.translation[2] = l_foot_initial.translation[2] + l_foot_traj['z'][i]

    dlfoot.translation[0] = l_foot_traj['dx'][i]
    dlfoot.translation[1] = l_foot_traj['dy'][i]
    dlfoot.translation[2] = l_foot_initial.translation[2] + l_foot_traj['dz'][i]


    # set com trajectory
    com.translation[0] = com_traj_x[i]
    com.translation[1] = com_traj_y[i]


    # TODO add velocity reference!
    ci_solver.sendTrajectory(com_task, pos=com, sim=0)
    ci_solver.sendTrajectory(ctrl_tasks[0], pos=lfoot,  sim=0) #vel=dlfoot,
#
#
#     # print('LEFT FOOT:')
#     # print('commanded:', lfoot.translation[0])
#     # print('sensed:', model.getPose(ctrl_tasks[0].getName()).translation)
#     #
#     # print('COM:')
#     # print('commanded:', com.translation[0])
#     # print('sensed:', model.getCOM())
#
#     # print('\n')
#
    t += 1./freq
    i += 1

    rate.sleep()







