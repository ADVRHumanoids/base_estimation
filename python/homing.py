from cartesian_interface import affine3 as a3

def homing(ci_solver, model, l_foot, r_foot, com, duration):

    # get x of robot
    x_l_foot = model.getPose(l_foot.getName()).translation[0]
    x_r_foot = model.getPose(r_foot.getName()).translation[0]

    print('x_r_foot', x_r_foot)

    com_initial = model.getCOM()

    com_correction = -com_initial[0] + x_r_foot
    print('com_correction: ', com_correction)

    goal_com = a3.Affine3()
    goal_com.translation = com_initial
    print('goal_com.translation', goal_com.translation[0])
    goal_com.translation[0] += com_correction

    ci_solver.reach(com, goal_com, duration, sim=1)