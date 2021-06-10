import numpy as np

def interpolator(traj_old, step_i, step_f, step_height, time, t_i, t_f, freq):
    # todo do something with the traj_old
    traj = dict()
    # print('traj_old', traj_old.shape)
    traj_len = (np.ceil(float(freq) * float(t_f - t_i))).astype(int)
    # print('traj_len', traj_len)
    traj_len_before = np.ceil(float(freq) * float(t_i)).astype(int)
    # print('traj_len_before', traj_len_before)
    traj_len_after = np.ceil(float(freq) * float(time-t_f)).astype(int) + 1 # todo for now is N+1 so traj_len_after lasts 1 node more
    # print('traj_len_after', traj_len_after)

    t = np.linspace(0, 1, np.ceil(traj_len))
    dt = 1. / float(freq)

    traj['x'] = np.full(traj_len_before, step_i[0])
    traj['y'] = np.full(traj_len_before, step_i[1])
    traj['z'] = np.full(traj_len_before, 0.)

    traj['dx'] = np.full(traj_len_before, 0.)
    traj['dy'] = np.full(traj_len_before, 0.)
    traj['dz'] = np.full(traj_len_before, 0.)

    traj['ddx'] = np.full(traj_len_before, 0.)
    traj['ddy'] = np.full(traj_len_before, 0.)
    traj['ddz'] = np.full(traj_len_before, 0.)

    traj['x'] = np.append(traj['x'], (step_i[0] + (((6. * t - 15.) * t + 10.) * t ** 3.) * (step_f[0] - step_i[0])))  # on the x
    traj['y'] = np.append(traj['y'], (step_i[1] + (((6. * t - 15.) * t + 10.) * t ** 3.) * (step_f[1] - step_i[1]))) # on the y
    traj['z'] = np.append(traj['z'], (64. * t ** 3. * (1. - t) ** 3.) * step_height) # on the z

    traj['x'] = np.append(traj['x'], np.full(traj_len_after, step_f[0]))
    traj['y'] = np.append(traj['y'], np.full(traj_len_after, step_f[1]))
    traj['z'] = np.append(traj['z'], np.full(traj_len_after, 0.))

    # compute velocity # todo remember this, derivative of function and then (t_f - t_i) to make it consistent with the parametrization
    traj['dx'] = np.append(traj['dx'], 30. * (t - 1.) ** 2. * t ** 2. * (step_f[0] - step_i[0])) / (t_f - t_i)
    traj['dy'] = np.append(traj['dy'], 30. * (t - 1.) ** 2. * t ** 2. * (step_f[1] - step_i[1])) / (t_f - t_i)
    traj['dz'] = np.append(traj['dz'], step_height * (t - 1.) ** 2. * (t ** 2.0 * (192. - 192. * t) - 192. * t ** 3.)) / (t_f - t_i)

    traj['dx'] = np.append(traj['dx'], np.full(traj_len_after, 0.))
    traj['dy'] = np.append(traj['dy'], np.full(traj_len_after, 0.))
    traj['dz'] = np.append(traj['dz'], np.full(traj_len_after, 0.))

    # compute acceleration
    # traj['ddx'] = np.append(traj['ddx'], 60 * t * (2 * t ** 2 - 3. * t + 1) * (step_f[0] - step_i[0]))
    # traj['ddy'] = np.append(traj['ddy'], 60 * t * (2 * t ** 2 - 3. * t + 1) * (step_f[1] - step_i[1]))
    # traj['ddz'] = np.append(traj['ddz'], step_height * (384.0 * t ** 1.0 - 2304.0 * t ** 2.0 + 3840.0 * t ** 3.0 - 1920.0 * t ** 4.0))
    #
    # traj['ddx'] = np.append(traj['ddx'], np.full(traj_len_after, 0.))
    # traj['ddy'] = np.append(traj['ddy'], np.full(traj_len_after, 0.))
    # traj['ddz'] = np.append(traj['ddz'], np.full(traj_len_after, 0.))

    # small hack for t filling
    t = np.linspace(0, 1, len(traj['x']))

    return t, traj