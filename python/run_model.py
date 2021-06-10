import numpy as np
import step_solver as ss
import plot_step as stpPlotter
from cartesian_interface import affine3 as a3
import rospy

def tryWithoutRobot(ci_solver, model, l_sole_task, r_sole_task, com_task, n_duration, initial_ds_t, single_stance_t, final_ds_t, freq, plot=0, unroll=0):

    T_tot = initial_ds_t + single_stance_t + final_ds_t

    initial_com_vel = 0.5
    height_com = model.getCOM()[2] - model.getPose(l_sole_task.getName()).translation[2]
    initial_com = np.array([[model.getCOM()[0], model.getCOM()[1]], [initial_com_vel, 0], [0., 0.]])
    initial_l_foot = np.array([model.getPose(l_sole_task.getName()).translation[0],
                               model.getPose(l_sole_task.getName()).translation[1], 0.])
    initial_r_foot = np.array([model.getPose(r_sole_task.getName()).translation[0],
                               model.getPose(r_sole_task.getName()).translation[1], 0.])

    solver = ss.StepSolver(n_duration, initial_ds_t, single_stance_t, final_ds_t, height_com)
    solver.buildProblemStep()

    # print('initial_com:', initial_com)
    # print('initial_l_foot:', initial_l_foot)
    # print('initial_r_foot:', initial_r_foot)
    # initial_l_foot = np.array([-0.128, 0.103, 0.])
    # initial_r_foot = np.array([-0.128, -0.103, 0.])
    # initial_com = np.array([[-0.067, 0.], [0.15, 0.], [0., 0.]])

    opt_values = solver.solveProblemStep(initial_com, initial_l_foot, initial_r_foot)

    opt_values['p'] = opt_values['x'][0:2, :]
    opt_values['v'] = opt_values['x'][2:4, :]
    opt_values['a'] = opt_values['x'][4:6, :]

    del opt_values['x']

    com_traj, l_foot_traj = solver.interpolateStep(initial_com, opt_values, initial_ds_t,
                                                   initial_ds_t + single_stance_t, freq)

    # PLOTTING =============================================
    if plot:
        stpPlotter.plotStep(opt_values, solver, com_traj, l_foot_traj)

    if unroll:

        lfoot = a3.Affine3()
        dlfoot = np.zeros([6, 1])

        com = a3.Affine3()
        com.translation = model.getCOM()
        dcom = np.zeros([6, 1])

        l_foot_h = model.getPose(l_sole_task.getName()).translation[2]
        lfoot.linear = model.getPose(l_sole_task.getName()).linear

        i = 0
        intial_t = 0
        t = 0
        rate = rospy.Rate(freq)

        while t < (intial_t + T_tot):
            lfoot.translation[0] = l_foot_traj['x'][i]
            lfoot.translation[1] = l_foot_traj['y'][i]
            lfoot.translation[2] = l_foot_h + l_foot_traj['z'][i]

            dlfoot[0] = l_foot_traj['dx'][i]
            dlfoot[1] = l_foot_traj['dy'][i]
            dlfoot[2] = l_foot_traj['dz'][i]

            com.translation[0] = com_traj['x'][i]
            com.translation[1] = com_traj['y'][i]

            dcom[0] = com_traj['dx'][i]
            dcom[1] = com_traj['dy'][i]

            com_task.setPoseReference(com)
            com_task.setVelocityReference(dcom)

            l_sole_task.setPoseReference(lfoot)
            l_sole_task.setVelocityReference(dlfoot)

            ci_solver.update(t, sim=False)

            t += 1. / freq
            i += 1
            rate.sleep()

    exit()