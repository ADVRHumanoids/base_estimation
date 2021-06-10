# /usr/bin/env python3

import matplotlib
matplotlib.use('TkAgg')
import run_model
import numpy as np

##### for robot and stuff
import xbot_interface.config_options as xbot_opt
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
from moveit_commander.roscpp_initializer import roscpp_initialize

from ci_solver import CartesianInterfaceSolver
from com_vel_est import FloatingBase
import send_force_gazebo as sfg
import homing
from xbot_msgs.msg import JointState
import rospy

import step_solver as ss

if __name__ == '__main__':

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
    pos_ref = [0, 0, 0, 0, 0, 0]  # fake floating base + position reference after homing
    pos_ref.extend(list(initial_joint_state.position_reference))

    robot.sense()

    # model.syncFrom(robot)
    model.setJointPosition(pos_ref)
    model.update()  # <--THIS IS NEEDED

    # set fake floating base
    world_T_l_sole = model.getPose('l_sole')
    w_T_fb = model.getFloatingBasePose()
    l_sole_T_fb = world_T_l_sole.inverse() * w_T_fb
    l_sole_T_fb.translation[1] += (model.getPose('l_sole').translation[1] - model.getPose('r_sole').translation[1]) / 2.
    model.setFloatingBasePose(l_sole_T_fb)

    # model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
    model.update()

    # CREATE CARTESIAN SOLVER
    ci_solver = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=1. / freq)
    print('Created cartesian interface.')
    ctrl_tasks = [None, None]

    ctrl_tasks[0], ctrl_tasks[1], l_arm_task, r_arm_task, com_task = ci_solver.getTasks()

    ##### HOMING OF ROBOT #####

    # robot.setPositionReference([0.,    -0.356,  0.,     0.75,  -0.34, 0.,     0.,    -0.356,  0.,     0.75,  -0.34,   0., -0.,    -0.,     1.017,
    # -0.008, -0.022 ,-1.934, -0.009, -0.571, -0.014,  1.017,  0.008,  0.022, -1.934, 0.009, -0.571,  0.014])
    # robot.move()
    # robot.sense()
    # model.syncFrom(robot)
    # model.update()
    homing.homing(ci_solver, model, ctrl_tasks[0], ctrl_tasks[1], com_task, 1.)

    print('l_foot_initial:', model.getPose(ctrl_tasks[0].getName()))
    print('r_foot_initial:', model.getPose(ctrl_tasks[1].getName()))
    print('com:', model.getCOM())

    # set real floating base
    model.syncFrom(robot)
    model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
    model.update()

    print('l_foot_initial:', model.getPose(ctrl_tasks[0].getName()))
    print('r_foot_initial:', model.getPose(ctrl_tasks[1].getName()))
    print('com:', model.getCOM())

    ci_solver_fb = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=1. / freq)
    print('Created feedback cartesian interface.')

    ctrl_tasks[0], ctrl_tasks[1], l_arm_task, r_arm_task, com_task = ci_solver_fb.getTasks()
    # ctrl_tasks, com_task = ci_solver_fb.getTasks()

    height_com = model.getCOM()[2] - model.getPose(ctrl_tasks[0].getName()).translation[2]
    ## CONSTRUCT OPTIMAL PROBLEM
    # run_model.tryWithoutRobot(ci_solver, model, ctrl_tasks[0], ctrl_tasks[1], com_task, n_duration, initial_ds_t, single_stance_t, final_ds_t, freq, plot=1, unroll=0)

    solver = ss.StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
    solver.buildProblemStep()

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
    threshold = 0.5
    flag_step = False

    t_start_step = None

    n_solve = 0
    max_solves = 1

    l_foot_initial = Affine3()
    r_foot_initial = Affine3()

    lfoot.linear = model.getPose(ctrl_tasks[0].getName()).linear

    com_task.setActivationState(pyci.ActivationState.Enabled)
    ctrl_tasks[0].setActivationState(pyci.ActivationState.Enabled)

    print('started spinning...')

    i = 0
    exit_loop = 1
    time_in_loop = 0
    force_flag = 1
    # interactive STEP
    while exit_loop:

        if force_flag and time_in_loop > 3.:
            print('PUSHED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            sfg.sendForceGazebo([500, 0., 0.], 0.1)
            force_flag = 0

        # model.syncFrom(robot) # todo remove or add?
        model.setFloatingBaseState(fb.getFbPose(), fb.getFbTwist())
        model.update()

        if model.getCOMVelocity()[0] >= threshold and not flag_step and n_solve < max_solves:
            initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [threshold, 0], [0., 0.]])
            # initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [model.getCOMVelocity()[0], model.getCOMVelocity()[1]], [0., 0.]])

            l_foot_initial.translation = model.getPose(ctrl_tasks[0].getName()).translation
            l_foot_initial.linear = model.getPose(ctrl_tasks[0].getName()).linear

            r_foot_initial.translation = model.getPose(ctrl_tasks[1].getName()).translation
            r_foot_initial.linear = model.getPose(ctrl_tasks[1].getName()).linear

            initial_l_foot = np.array([l_foot_initial.translation[0], l_foot_initial.translation[1], 0.])
            initial_r_foot = np.array([r_foot_initial.translation[0], r_foot_initial.translation[1], 0.])

            print('initial_l_foot:', initial_l_foot)
            print('initial_r_foot:', initial_r_foot)
            print('com pos sensed:', model.getCOM())
            print('com vel sensed:', model.getCOMVelocity())
            # solve problem with given constraints
            opt_values = solver.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)
            # interpolate at right frequency

            com_traj, l_foot_traj = solver.interpolateStep(initial_com, opt_values, initial_ds_t,
                                                           initial_ds_t + single_stance_t, freq)

            n_solve += 1
            flag_step = True
            t_start_step = t

        if flag_step:

            # set l_foot trajectory
            lfoot.translation[0] = l_foot_traj['x'][i]
            lfoot.translation[1] = l_foot_traj['y'][i]
            lfoot.translation[2] = l_foot_traj['z'][i]

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
                exit_loop = 0

            t += 1. / freq

        ci_solver_fb.update(t, sim=True)
        time_in_loop += 1. / freq
        rate.sleep()