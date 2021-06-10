import casadi
import pprint
from python.old_stuff import step
import python.step_solver as ss
import numpy as np
import rospy
from moveit_commander.roscpp_initializer import roscpp_initialize
import xbot_interface.config_options as xbot_opt
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from python.com_vel_est import FloatingBase
from xbot_msgs.msg import JointState
import time

np.set_printoptions(precision=3, suppress=True)

rospy.init_node('step')
# roscpp_init('step', [])
roscpp_initialize('step')

## optimal problem variables
initial_ds_t = 0.04
single_stance_t = 0.5
final_ds_t = 0.4
T_tot = initial_ds_t + single_stance_t + final_ds_t

n_duration = 0.02
freq = 100.

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

# add floating base estimation
fb = FloatingBase()
initial_joint_state = rospy.wait_for_message('/xbotcore/joint_states', JointState)
pos_ref = [0, 0, 0, 0, 0, 0] # fake floating base + position reference after homing
pos_ref.extend(list(initial_joint_state.position_reference))

robot.sense()

# model.syncFrom(robot)
model.setJointPosition(pos_ref)
model.update() #<--THIS IS NEEDED

# set fake floating base
world_T_l_sole = model.getPose('l_sole')
w_T_fb = model.getFloatingBasePose()
l_sole_T_fb = world_T_l_sole.inverse()*w_T_fb
l_sole_T_fb.translation[1] += (model.getPose('l_sole').translation[1] - model.getPose('r_sole').translation[1])/2.
model.setFloatingBasePose(l_sole_T_fb)

# model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
model.update()


T_tot = initial_ds_t + single_stance_t + final_ds_t

initial_com_vel = 0.5
height_com = model.getCOM()[2] - model.getPose('l_sole').translation[2]
initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [initial_com_vel, 0], [0., 0.]])
initial_l_foot = np.array([model.getPose('l_sole').translation[0], model.getPose('l_sole').translation[1], 0.])
initial_r_foot = np.array([model.getPose('r_sole').translation[0], model.getPose('r_sole').translation[1], 0.])



print('================================= STARTING TO SOLVE WITH NEW HORIZON ============================================================')
t = time.time()
################################# HORIZON #################################
solver = ss.StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
solver.buildProblemStep()
t_for_build_new = time.time() - t

t = time.time()
opt_values = solver.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)
solution = solver.prb.w_opt
t_for_solve_new = time.time() - t

opt_values['p'] = opt_values['x'][0:2, :]
opt_values['v'] = opt_values['x'][2:4, :]
opt_values['a'] = opt_values['x'][4:6, :]

del opt_values['x']

print('================================= STARTING TO SOLVE WITH OLD HORIZON ============================================================')
t = time.time()
################################# HORIZON OLD #################################
solver_old = step.StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
solver_old.buildProblemStep()
t_for_build_old = time.time() - t

opt_values_old = solver_old.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)
solution_old = solver_old.w_opt
t_for_solve_old = time.time() - t

prob = solver.prb.prob
prob_old = solver_old.prb.prob

# print(prob['f'])
# print(prob_old['f'])
if str(prob['f']) == str(prob_old['f']):
    print('f is equal')
# print(prob['x'])
# print(prob_old['x'])
if str(prob['x']) == str(prob_old['x']):
    print('x is equal')
# print(prob['g'])
# print(prob_old['g'])
if str(prob['g']) == str(prob_old['g']):
    print('g is equal')

if (solver.prb.w0 == solver_old.prb.w0).all():
    print('w0 is equal')

if (solver.prb.lbw == solver_old.prb.lbw):
    print('lbw is equal')

if (solver.prb.ubw == solver_old.prb.ubw):
    print('ubw is equal')

if (solver.prb.lbg == solver_old.prb.ct.lbg):
    print('lbg is equal')

if (solver.prb.ubg == solver_old.prb.ct.ubg):
    print('ubg is equal')

# print('ubg', solver.prb.ubg)
# print('ubg', solver_old.prb.ct.ubg)

# print('p', opt_values['p'])
# print('p_old', opt_values_old['p'])
#
# pprint.pprint(solution)
# print('=====================')
# pprint.pprint(solution_old)
#
#
if (solution == solution_old).all():
    print('solution is EQUAL')

if (opt_values['p'] == opt_values_old['p']).all():
    print('p is EQUAL')

print('TIME COMPARISON:')
print('build:')
print('old --> {}'.format(t_for_build_old))
print('new --> {}'.format(t_for_build_new))
print('solve:')
print('old --> {}'.format(t_for_solve_old))
print('new --> {}'.format(t_for_solve_new))

# Number of nonzeros in equality constraint Jacobian...:     2660
# Number of nonzeros in inequality constraint Jacobian.:        0
# Number of nonzeros in Lagrangian Hessian.............:      924
#
# Total number of variables............................:      944
#                      variables with only lower bounds:        0
#                 variables with lower and upper bounds:      478
#                      variables with only upper bounds:        0
# Total number of equality constraints.................:      589
# Total number of inequality constraints...............:        0
#         inequality constraints with only lower bounds:        0
#    inequality constraints with lower and upper bounds:        0
#         inequality constraints with only upper bounds:        0



# Number of objective function evaluations             = 28
# Number of objective gradient evaluations             = 27
# Number of equality constraint evaluations            = 28
# Number of inequality constraint evaluations          = 0
# Number of equality constraint Jacobian evaluations   = 27
# Number of inequality constraint Jacobian evaluations = 0
# Number of Lagrangian Hessian evaluations             = 26
# Total CPU secs in IPOPT (w/o function evaluations)   =      0.103
# Total CPU secs in NLP function evaluations           =      0.005