import matplotlib.pyplot as plt
import numpy as np
import casadi as cs

def plotStep(opt_values, solver, com_traj, l_foot_traj):

    cp = opt_values['p'] + opt_values['v'] / np.sqrt(solver.grav / solver.height_com)
    plt.figure()
    plt.clf()
    # ######## COM ######
    plt.scatter(opt_values['p'][0, 0], opt_values['p'][1, 0], color='r', edgecolors='r', s=60)
    plt.scatter(opt_values['p'][0, -1], opt_values['p'][1, -1], color='r', edgecolors='k', s=60)
    plt.plot(opt_values['p'][0], opt_values['p'][1], linewidth=2)
    # ######## CP ######
    plt.scatter(cp[0, 0], cp[1, 0], color='m', edgecolors='m', s=60)
    plt.scatter(cp[0, -1], cp[1, -1], color='m', edgecolors='k', s=60)
    plt.plot(cp[0, :], cp[1, :], color='m', linewidth=2, ls=':')
    # ######## ZMP ######
    zmp = list(np.array(opt_values['p']) - np.array(opt_values['a']) * (solver.height_com / solver.grav))
    plt.plot(zmp[0], zmp[1], linewidth=1)

    alphal = np.sum(opt_values['alpha_l'], 0)
    alphar = np.sum(opt_values['alpha_r'], 0)

    ######## LEFT foot ######
    k = 0
    for pos_x, pos_y in zip(opt_values['l'][0], opt_values['l'][1]):
        if alphal[k] > 1e-6:
            coord = solver.getEdges([pos_x, pos_y])
            coord = cs.vertcat(coord, coord[0, :])
            plt.plot(coord[:, 0], coord[:, 1], color='c')
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

    ######## lines connecting the steps ######
    plt.plot([opt_values['l'][0, 0], opt_values['r'][0, 0]], [opt_values['l'][1, 0], opt_values['r'][1, 0]], 'k--')
    # plt.plot([opt_values['l'][0, -1], opt_values['r'][0, -1]], [opt_values['l'][1, -1], opt_values['r'][1, -1]], 'k--')

    plt.legend(['com', 'cp', 'zmp'], loc=3)

    ####### ALPHA feet ######
    plt.figure()
    tgrid = [solver.T_total / solver.N * k for k in range(solver.N + 1)]
    plt.title('alpha  sum')
    plt.scatter(tgrid, alphal)
    plt.scatter(tgrid, alphar, color='r')
    plt.legend(['l', 'r'])
    plt.xlabel('t')
    plt.ylabel('weigth')

    ###### com and foot traj in time ##########
    tgrid_interp = [solver.T_total / (solver.N * 2) * k for k in range((solver.N * 2) + 1)]

    plt.figure()
    plt.title('position')
    plt.plot(tgrid, opt_values['p'][0, :], 'r--', linewidth=2)
    plt.plot(tgrid_interp, com_traj['x'], 'r', linewidth=2)
    plt.plot(tgrid, opt_values['l'][0, :], 'b--', linewidth=2)
    plt.plot(tgrid_interp, l_foot_traj['x'], 'b', linewidth=2)
    plt.legend(['com_opt', 'com', 'foot_opt', 'foot'], loc=3)

    plt.figure()
    plt.title('velocity')
    plt.plot(tgrid, opt_values['v'][0, :], 'r--', linewidth=2)
    plt.plot(tgrid_interp, com_traj['dx'], 'r', linewidth=2)
    plt.plot(tgrid_interp, l_foot_traj['dx'], 'b', linewidth=2)
    plt.legend(['com_vel_opt', 'com_vel', 'foot_vel'])

    ###### com traj in time ##########

    plt.figure()
    plt.title('com_x')
    plt.plot(tgrid, opt_values['p'][0, :], linewidth=2)
    plt.plot(tgrid, opt_values['v'][0, :], linewidth=2)
    plt.plot(tgrid, opt_values['a'][0, :], linewidth=2)
    plt.plot(tgrid[:-1], opt_values['u'][0, :], linewidth=2)

    plt.figure()

    plt.title('com_y')
    plt.plot(tgrid, opt_values['p'][0, :], linewidth=2)
    plt.plot(tgrid, opt_values['v'][0, :], linewidth=2)
    plt.plot(tgrid, opt_values['a'][0, :], linewidth=2)
    plt.plot(tgrid[:-1], opt_values['u'][0, :], linewidth=2)

    plt.show()