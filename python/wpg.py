#/usr/bin/env python2.7

from casadi import *
from numpy import *

def getWeightedEdges(alpha_vec, p, length_foot, width_foot):
    lu = [alpha_vec[0] * (p[0] + length_foot / 2.), alpha_vec[0] * (p[1] + width_foot / 2.)]
    ru = [alpha_vec[1] * (p[0] + length_foot / 2.), alpha_vec[1] * (p[1] - width_foot / 2.)]
    rh = [alpha_vec[2] * (p[0] - length_foot / 2.), alpha_vec[2] * (p[1] - width_foot / 2.)]
    lh = [alpha_vec[3] * (p[0] - length_foot / 2.), alpha_vec[3] * (p[1] + width_foot / 2.)]
    return lu, ru, rh, lh


N = 50 # number of control intervals
T = 2.0 # time horizon

width_foot = 0.1
length_foot = 0.2

max_stride_x = 0.8
max_stride_y = width_foot / 2. + 0.5
min_stride_y = width_foot / 2. + 0.1

grav = 9.81
h = 1.

# state variables
p = MX.sym('p', 2) # com position
v = MX.sym('v', 2) # com velocity
a = MX.sym('a', 2) # com acceleration

l = MX.sym('l', 2) # left foot
r = MX.sym('r', 2) # right foot

alpha_l = MX.sym('alpha_l', 4) # left foot weigths [lu, ru, rh, lh]
alpha_r = MX.sym('alpha_r', 4) # right foot weigths [lu, ru, rh, lh]

x = vertcat(p, v, a) #, l, r, alpha_l, alpha_r) # state



# control variables
j = MX.sym('j', 2) # com jerk
u = j # control

# model equation
xdot = vertcat(v, a, j)

# Objective terms
L = sumsqr(u)
# Formulate discrete time dynamics
# Fixed step Runge-Kutta 4 integrator
M = 1 # RK4 steps per interval
DT = T / N / M
f = Function('f', [x, u], [xdot, L])

X0 = MX.sym('X0', x.size()[0])
U = MX.sym('U', u.size()[0])
X = X0
Q = 0
for j in range(M):
   k1, k1_q = f(X, U)
   k2, k2_q = f(X + DT/2 * k1, U)
   k3, k3_q = f(X + DT/2 * k2, U)
   k4, k4_q = f(X + DT * k3, U)
   X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
   Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

z = vertcat(p, v, a, l, r, alpha_l, alpha_r) # state

# Start with an empty NLP
w=[] # optimization variables along all the knots
w0 = [] # initial value for optimization variables
lbw = [] # lower bound on variables
ubw = [] # upper bound on variables
J = 0 # const funciton
g=[] # constraints
lbg = [] # lower constraint
ubg = [] # upper constraints

XInt = []

# main loop
for k in range(N+1):
    # STATE
    Xk = MX.sym('X_' + str(k), z.size()[0])
    w += [Xk]
    if k == 0: # at the beginning, position, velocity and acceleration to ZERO
        lbw += [0., 0.,  # com pos
                0., 0.,  # com vel
                0., 0.,  # com acc
                0., 0.1,  # left foot pos
                0., -0.1,  # left foot pos
                0., 0., 0., 0.,  # alpha l
                0., 0., 0., 0.  # alpha r
                ]
        ubw += [0., 0.,  # com pos
                0., 0.,  # com vel
                0., 0.,  # com acc
                0., 0.1,  # left foot pos
                0., -0.1,  # left foot pos
                1., 1., 1., 1.,  # alpha l
                1., 1., 1., 1.  # alpha r
                ]
    elif k == N:
        lbw += [-100., -100.,  # com pos
                0., 0.,  # com vel
                0., 0.,  # com acc
                -100., -100.,  # left foot pos
                -100., -100.,  # right foot pos
                0., 0., 0., 0.,
                0., 0., 0., 0.,
                #0.25, 0.25, 0.25, 0.25,  # alpha l
                #0.25, 0.25, 0.25, 0.25  # alpha r
                ]
        ubw += [100., 100.,  # com pos
                0., 0.,  # com vel
                0., 0.,  # com acc
                100., 100.,  # left foot pos
                100., 100.,  # right foot pos
                1., 1., 1., 1.,
                1., 1., 1., 1.,
                # 0.25, 0.25, 0.25, 0.25,  # alpha l
                # 0.25, 0.25, 0.25, 0.25  # alpha r
                ]
    else:
        lbw += [-100., -100., # com pos
                -100., -100., # com vel
                -100., -100., # com acc
                -100., -100., # left foot pos
                -100., -100., # right foot pos
                0., 0., 0., 0.,  # alpha l
                0., 0., 0., 0.  # alpha r
                ]
        ubw += [100., 100., # com pos
                100., 100., # com vel
                100., 100., # com acc
                100., 100., # left foot pos
                100., 100., # right foot pos
                1., 1., 1., 1.,  # alpha l
                1., 1., 1., 1.  # alpha r
                ]

    w0 += list(np.zeros(z.size()[0]))

    Pk = Xk[0:2]
    Ak = Xk[4:6]

    if k < N:
        # CONTROL
        Uk = MX.sym('U_' + str(k), u.size()[0])
        w += [Uk]

        lbw += [-1000., -1000.]
        ubw += [1000., 1000.]

        w0 += list(np.zeros(u.size()[0]))

        # Integrator
        Fk = F(x0=Xk[0:6], p=Uk)
        XInt.append([Fk['xf'], Xk[6:8], Xk[8:10]])
        J = J + mtimes(Fk['qf'].T, Fk['qf'])

        # Regularization of alphas
        Ak = Xk[10:18]
        J = J + mtimes(Ak.T, Ak)


    #WALKING SCHEDULER
    ZMP = Xk[0:2] - Xk[4:6] * (h / grav)
    Lk = Xk[6:8]
    Rk = Xk[8:10]


    g += [Lk[0] - Rk[0]]
    lbg += [-max_stride_x]
    ubg += [max_stride_x]
    g += [Lk[1] - Rk[1]]
    lbg += [-max_stride_y]
    ubg += [max_stride_y]

    J = J + 10*sumsqr((Lk[1] - Rk[1]) - min_stride_y)


    l_lu, l_ru, l_rh, l_lh = getWeightedEdges(Xk[10:14], Lk, length_foot, width_foot)
    r_lu, r_ru, r_rh, r_lh = getWeightedEdges(Xk[14:18], Rk, length_foot, width_foot)
    if k <= 20: #double stance
        # ZMP in support polygon
        for i in range(2):
            g += [ZMP[i] - (l_lu[i] + l_ru[i] + l_rh[i] + l_lh[i] + r_lu[i] + r_ru[i] + r_rh[i] + r_lh[i])]
            lbg += [0]
            ubg += [0]

        g += [Xk[10] + Xk[11] + Xk[12] + Xk[13] + Xk[14] + Xk[15] + Xk[16] + Xk[17]]
        lbg += [1.]
        ubg += [1.]

        if k > 0:
            Lk_prev = XInt[k - 1][1]
            Rk_prev = XInt[k - 1][2]
            g += [Lk - Lk_prev]
            lbg += [0., 0.]
            ubg += [0., 0.]
            g += [Rk - Rk_prev]
            lbg += [0., 0.]
            ubg += [0., 0.]

    #elif k > 20 and k <= 50:
    else:
        J += 10000 * mtimes((ZMP - [0.5, 0.5]).T, (ZMP - [0.5, 0.5]))

        if k <= 30: #single stance
            for i in range(2):
                g += [ZMP[i] - (r_lu[i] + r_ru[i] + r_rh[i] + r_lh[i])]
                lbg += [0]
                ubg += [0]

            g += [Xk[10] + Xk[11] + Xk[12] + Xk[13]]
            lbg += [0.]
            ubg += [0.]
            g += [Xk[14] + Xk[15] + Xk[16] + Xk[17]]
            lbg += [1.]
            ubg += [1.]

            Rk_prev = XInt[k - 1][2]
            g += [Rk - Rk_prev]
            lbg += [0., 0.]
            ubg += [0., 0.]

        else: #double stance
            # ZMP in support polygon
            for i in range(2):
                g += [ZMP[i] - (l_lu[i] + l_ru[i] + l_rh[i] + l_lh[i] + r_lu[i] + r_ru[i] + r_rh[i] + r_lh[i])]
                lbg += [0]
                ubg += [0]

            g += [Xk[10] + Xk[11] + Xk[12] + Xk[13] + Xk[14] + Xk[15] + Xk[16] + Xk[17]]
            lbg += [1.]
            ubg += [1.]

            if k > 0:
                Lk_prev = XInt[k - 1][1]
                Rk_prev = XInt[k - 1][2]
                g += [Lk - Lk_prev]
                lbg += [0., 0.]
                ubg += [0., 0.]
                g += [Rk - Rk_prev]
                lbg += [0., 0.]
                ubg += [0., 0.]

    # elif k > 50 and k <= 80:
    #     J += 100 * mtimes((ZMP - [1., -0.5]).T, (ZMP - [1., -0.5]))
    #
    #     for i in range(2):
    #         g += [ZMP[i] - (r_lu[i] + r_ru[i] + r_rh[i] + r_lh[i])]
    #         lbg += [0]
    #         ubg += [0]
    #
    #     g += [Xk[10] + Xk[11] + Xk[12] + Xk[13]]
    #     lbg += [0.]
    #     ubg += [0.]
    #     g += [Xk[14] + Xk[15] + Xk[16] + Xk[17]]
    #     lbg += [1.]
    #     ubg += [1.]
    #
    #     if k > 0:
    #         Lk_prev = XInt[k - 1][1]
    #         Rk_prev = XInt[k - 1][2]
    #         g += [Rk - Rk_prev]
    #         lbg += [0., 0.]
    #         ubg += [0., 0.]
    #
    # elif k > 80:
    #     J += 100 * mtimes((ZMP - [1., -0.5]).T, (ZMP - [1., -0.5]))
    #
    #     for i in range(2):
    #         g += [ZMP[i] - (l_lu[i] + l_ru[i] + l_rh[i] + l_lh[i])]
    #         lbg += [0]
    #         ubg += [0]
    #     for i in range(2):
    #         g += [ZMP[i] - (r_lu[i] + r_ru[i] + r_rh[i] + r_lh[i])]
    #         lbg += [0]
    #         ubg += [0]
    #
    #     g += [Xk[10] + Xk[11] + Xk[12] + Xk[13]]
    #     lbg += [1.]
    #     ubg += [1.]
    #     g += [Xk[14] + Xk[15] + Xk[16] + Xk[17]]
    #     lbg += [1.]
    #     ubg += [1.]
    #
    #     if k > 0:
    #         Lk_prev = XInt[k - 1][1]
    #         Rk_prev = XInt[k - 1][2]
    #         g += [Lk - Lk_prev]
    #         lbg += [0., 0.]
    #         ubg += [0., 0.]
    #
    #         g += [Rk - Rk_prev]
    #         lbg += [0., 0.]
    #         ubg += [0., 0.]



    # Multiple Shooting (the result of the integrator [XInt[k-1]] must be the equal to the value of the next node)
    if k > 0:
        g += [XInt[k-1][0] - Xk[0:6]]
        lbg += list(np.zeros(x.size()[0]))
        ubg += list(np.zeros(x.size()[0]))





# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob)

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()

# Plot the solution
p_opt = []
v_opt = []
a_opt = []
j_opt = []
l_opt = []
r_opt = []
alpha_l_opt = []
alpha_r_opt = []

num_var = z.size()[0] + u.size()[0]
for k in range(N):
    sol_k = w_opt[num_var*k:(num_var*k+num_var)]
    p_opt = p_opt + list(sol_k[0:2])
    v_opt = v_opt + list(sol_k[2:4])
    a_opt = a_opt + list(sol_k[4:6])
    l_opt = l_opt + list(sol_k[6:8])
    r_opt = r_opt + list(sol_k[8:10])
    alpha_l_opt = alpha_l_opt + list(sol_k[10:14])
    alpha_r_opt = alpha_r_opt + list(sol_k[14:18])
    j_opt = j_opt + list(sol_k[18:20])

sol_k = w_opt[-z.size()[0]:]
p_opt = p_opt + list(sol_k[0:2])
v_opt = v_opt + list(sol_k[2:4])
a_opt = a_opt + list(sol_k[4:6])
l_opt = l_opt + list(sol_k[6:8])
r_opt = r_opt + list(sol_k[8:10])
alpha_l_opt = alpha_l_opt + list(sol_k[10:14])
alpha_r_opt = alpha_r_opt + list(sol_k[14:18])


tgrid = [T/N*k for k in range(N+1)]
px = p_opt[0::2]
vx = v_opt[0::2]
ax = a_opt[0::2]
lx = l_opt[0::2]
rx = r_opt[0::2]
jx = j_opt[0::2]



zmpx = list(np.array(px) - np.array(ax)*(h/grav))

alphal = [sum(alpha_l_opt[x:x+4]) for x in range(0, len(alpha_l_opt),4)]
alphar = [sum(alpha_r_opt[x:x+4]) for x in range(0, len(alpha_r_opt),4)]

jx.insert(0,float('nan'))
# zmpx.append(float('nan'))


import matplotlib.pyplot as plt
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


zmpy = list(np.array(py) - np.array(ay)*(h/grav))
jy.insert(0,float('nan'))
# zmpy.append(float('nan'))


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
        coord = [[pos_x+length_foot/2.,pos_y+width_foot/2.],
                 [pos_x+length_foot/2.,pos_y-width_foot/2.],
                 [pos_x-length_foot/2.,pos_y-width_foot/2.],
                 [pos_x-length_foot/2.,pos_y+width_foot/2.]]
        coord.append(coord[0]) #repeat the first point to create a 'closed loop'
        xs, ys = zip(*coord) #create lists of x and y values
        plt.plot(xs,ys, color='b')
        plt.scatter(pos_x, pos_y)
    k += 1

######## RIGHT foot ######
k = 0
for pos_x, pos_y in zip(rx, ry):
    if alphar[k] > 1e-6:
        coord = [[pos_x+length_foot/2.,pos_y+width_foot/2.], [pos_x+length_foot/2.,pos_y-width_foot/2.], [pos_x-length_foot/2.,pos_y-width_foot/2.], [pos_x-length_foot/2.,pos_y+width_foot/2.]]
        coord.append(coord[0]) #repeat the first point to create a 'closed loop'
        xs, ys = zip(*coord) #create lists of x and y values
        plt.plot(xs,ys, color='r')
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
