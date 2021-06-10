from ci_solver import CartesianInterfaceSolver
from wpg_old import Stepper
import numpy as np
import casadi as cs

import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import yaml

import matplotlib.pyplot as plt

np.set_printoptions(precision=3, suppress=True)

rospy.init_node('step_top')
roscpp_init('step_top', [])

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
freq = 100
ctrl_points = {0: 'l_sole', 1: 'r_sole'}
ci_solver = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=1./freq, ctrl_points=ctrl_points)
print 'Created cartesian interface.'

ctrl_tasks, com_task = ci_solver.getTasks()

l_foot_initial = Affine3()
l_foot_initial.translation = model.getPose(ctrl_tasks[0].getName()).translation
l_foot_initial.linear = model.getPose(ctrl_tasks[0].getName()).linear

r_foot_initial = Affine3()
r_foot_initial.translation = model.getPose(ctrl_tasks[1].getName()).translation
r_foot_initial.linear = model.getPose(ctrl_tasks[1].getName()).linear

com_initial = model.getCOM()

# ============================================== #

initial_ds = 0.2
ss_1 = 0.5
ds_1 = 0.
ss_2 = 0.
final_ds = 0.4
T_total = initial_ds + ss_1 + ds_1 + ss_2 + final_ds

stp = Stepper(initial_ds, ss_1, ds_1, ss_2, final_ds)

initial_l_foot = np.array([l_foot_initial.translation[0], l_foot_initial.translation[1], 0.])
initial_r_foot = np.array([r_foot_initial.translation[0], r_foot_initial.translation[1], 0.])

print('initial_l_foot:', initial_l_foot)
print('initial_r_foot:', initial_r_foot)

heigth_com = model.getCOM()[2] - model.getPose(ctrl_tasks[0].getName()).translation[2]

initial_com = np.array([[model.getCOM()[0],  model.getCOM()[1]], [0.2, 0.], [0., 0.]])

print('initial_com:', initial_com)

problem, initial_guess, constraints = stp.generateProblem(initial_com=initial_com, heigth_com=heigth_com, l_foot=initial_l_foot, r_foot=initial_r_foot)

w_opt = stp.solve(problem, initial_guess, constraints)

# stp.plot(w_opt)

com_states, feet_states = stp.get_relevant_variables(w_opt)

## SHOW INITIAL AND FINAL POSITIONS OF FEET
# for i in range(len(feet_states)):
#     plt.scatter(feet_states[i]['l'][0], feet_states[i]['l'][1])
#     plt.scatter(feet_states[i]['r'][0], feet_states[i]['r'][1])
# plt.show()
# exit()
# interpolating stuff
com_interpol = cs.DM(initial_com.flatten())

multiplier = 2
dt_int = stp.T / stp.N / 1 / multiplier
integrator = stp.RK4(1, 1, dt_int)
#

for i in range(len(com_states)):
    for multi_i in range(multiplier):
        Fk = integrator(x0=com_interpol[:, -1], p=com_states[i]['j'])
        com_interpol = cs.horzcat(com_interpol, Fk['xf'])


com_traj_x = com_interpol[0, :].full().flatten()
com_traj_y = com_interpol[1, :].full().flatten()


t, l_foot_traj = stp.interpolator(step_i=feet_states[0]['l'], step_f=feet_states[1]['l'], step_height=0.05, time=T_total, t_i =initial_ds, t_f=initial_ds+ss_1, freq=freq)

lfoot = Affine3()
lfoot.translation = l_foot_initial.translation
lfoot.linear = l_foot_initial.linear

dlfoot = Affine3()


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


    # print('LEFT FOOT:')
    # print('commanded:', lfoot.translation[0])
    # print('sensed:', model.getPose(ctrl_tasks[0].getName()).translation)
    #
    # print('COM:')
    # print('commanded:', com.translation[0])
    # print('sensed:', model.getCOM())

    # print('\n')

    t += 1./freq
    i += 1

    rate.sleep()

