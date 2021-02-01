#/usr/bin/env python2.7

from casadi import *
from numpy import *

def casadi_sum(self, axis=None, out=None):
    assert out is None
    if axis== 0:
        return sum1(self)
    elif axis== 1:
        return sum2(self)
    elif axis is None:
        return sum1(sum2(self))
    else:
        raise Exception("Invalid argument for sum")

def getEdges(p, length_foot, width_foot): # lu, ru, rh, lh

    lu = DM([+ length_foot / 2., + width_foot / 2.])
    ru = DM([+ length_foot / 2., - width_foot / 2.])
    rh = DM([- length_foot / 2., - width_foot / 2.])
    lh = DM([- length_foot / 2., + width_foot / 2.])

    ft_vert = horzcat(p + lu, p + ru, p + rh, p + lh).T

    return ft_vert

"""RK4 Runge-Kutta 4 integrator
Input:  
    L: objective fuction to integrate
    M: RK steps
    T: final time
    N: numbr of shooting nodes:
    x: state varibales
    u: controls
"""
def RK4(L, M, T, N, x, u):
    DT = T / N / M
    f = Function('f', [x, u], [xdot, L])
    X0 = sym_c.sym('X0', x.size()[0])
    U = sym_c.sym('U', u.size()[0])
    X = X0
    Q = 0

    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + DT / 2 * k1, U)
        k3, k3_q = f(X + DT / 2 * k2, U)
        k4, k4_q = f(X + DT * k3, U)
        X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
    return Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

def addConstraint(g, lbg, ubg, Gk, LBGk, UBGk):
    g += [Gk]
    lbg += LBGk
    ubg += UBGk
    return g, lbg, ubg

def addStance(g, lbg, ubg, ZMP, wft_r_vert, wft_l_vert, Xk, Lk, Rk, k, stance):
    if stance == 'L':
        g, lbg, ubg = addConstraint(g, lbg, ubg, ZMP - casadi_sum(wft_r_vert, 0).T, [0, 0], [0, 0])

        g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[10:14], 0).T, [0], [0])
        g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[14:18], 0).T, [1.], [1.])

        Rk_prev = XInt[k - 1][2]
        g, lbg, ubg = addConstraint(g, lbg, ubg, Rk - Rk_prev, [0, 0], [0, 0])

        return g, lbg, ubg

    # g, lbg, ubg = addConstraint(g, lbg, ubg, ZMP - casadi_sum(wft_r_vert, 0).T, [0, 0], [0, 0])
    #
    # g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[10:14], 0).T, [0], [0])
    # g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[14:18], 0).T, [1.], [1.])
    #
    # Rk_prev = XInt[k - 1][2]
    # g, lbg, ubg = addConstraint(g, lbg, ubg, Rk-Rk_prev, [0,0], [0,0])

    elif stance == 'R':
        g, lbg, ubg = addConstraint(g, lbg, ubg, ZMP - casadi_sum(wft_l_vert, 0).T, [0, 0], [0, 0])

        g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[10:14], 0).T, [1.], [1.])
        g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[14:18], 0).T, [0.], [0.])

        Lk_prev = XInt[k - 1][1]
        g, lbg, ubg = addConstraint(g, lbg, ubg, Lk - Lk_prev, [0, 0], [0, 0])

        return g, lbg, ubg

    elif stance == 'D':

        g, lbg, ubg = addConstraint(g, lbg, ubg, ZMP - (casadi_sum(wft_l_vert, 0).T + casadi_sum(wft_r_vert, 0).T), [0, 0], [0, 0])
        g, lbg, ubg = addConstraint(g, lbg, ubg, casadi_sum(Xk[10:18], 0), [1.], [1.])

        if k > 0:
            Lk_prev = XInt[k - 1][1]
            Rk_prev = XInt[k - 1][2]
            g, lbg, ubg = addConstraint(g, lbg, ubg, Lk - Lk_prev, [0, 0], [0, 0])
            g, lbg, ubg = addConstraint(g, lbg, ubg, Rk - Rk_prev, [0, 0], [0, 0])

        return g, lbg, ubg


#######################################


N = 50 # number of control intervals
T = 1.0 # time horizon

width_foot = 0.1
length_foot = 0.2

max_stride_x = 0.8
max_stride_y = width_foot / 2. + 0.5
min_stride_y = width_foot / 2. + 0.15

grav = 9.81
h = 1.

sym_c = MX
# state variables
p = sym_c.sym('p', 2) # com position
v = sym_c.sym('v', 2) # com velocity
a = sym_c.sym('a', 2) # com acceleration

l = sym_c.sym('l', 2) # left foot
r = sym_c.sym('r', 2) # right foot

alpha_l = sym_c.sym('alpha_l', 4) # left foot weigths [lu, ru, rh, lh]
alpha_r = sym_c.sym('alpha_r', 4) # right foot weigths [lu, ru, rh, lh]

x = vertcat(p, v, a) #, l, r, alpha_l, alpha_r) # state

# control variables
j = sym_c.sym('j', 2) # com jerk
u = j # control

# model equation
xdot = vertcat(v, a, j)

# Objective terms
L = sumsqr(u)
# Formulate discrete time dynamics
# Fixed step Runge-Kutta 4 integrator
M = 1 # RK4 steps per interval
F = RK4(L, M, T, N, x, u)

z = vertcat(p, v, a, l, r, alpha_l, alpha_r) # state

# Start with an empty NLP
w=[] # optimization variables along all the knots
w0 = [] # initial value for optimization variables
lbw = [] # lower bound on variables
ubw = [] # upper bound on variables
J = 0 # const function
g=[] # constraints
lbg = [] # lower constraint
ubg = [] # upper constraints

XInt = []
# main loop
for k in range(N+1):
    # STATE
    Xk = sym_c.sym('X_' + str(k), z.size()[0])
    w += [Xk]
    if k == 0: # at the beginning, position, velocity and acceleration to ZERO
        lbw += [0., 0.,  # com pos
                -.5, .0,  # com vel
                0., 0.,  # com acc
                0., 0.1,  # left foot pos
                0., -0.1,  # left foot pos
                0., 0., 0., 0.,  # alpha l
                0., 0., 0., 0.  # alpha r
                ]
        ubw += [0., 0.,  # com pos
                -.5, .0,  # com vel
                0., 0.,  # com acc
                0., 0.1,  # left foot pos
                0., -0.1,  # left foot pos
                1., 1., 1., 1.,  # alpha l
                1., 1., 1., 1.  # alpha r
                ]
    elif k == N: # final state
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
        Uk = sym_c.sym('U_' + str(k), u.size()[0])
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

    g, lbg, ubg = addConstraint(g, lbg, ubg, Lk - Rk, [-max_stride_x, -max_stride_y], [max_stride_x, max_stride_y])

    J = J + 1000.*sumsqr((Lk[1] - Rk[1]) - min_stride_y)

    # get weighted edges
    wft_l_vert = Xk[10:14] * getEdges(Lk, length_foot, width_foot)  # lu, ru, rh, lh
    wft_r_vert = Xk[14:18] * getEdges(Rk, length_foot, width_foot)  # lu, ru, rh, lh

    if k <= 20: #double stance
        g, lbg, ubg = addStance(g, lbg, ubg, ZMP, wft_r_vert, wft_l_vert, Xk, Lk, Rk, k, 'D')

    else:

        if k <= 30: #single stance
            g, lbg, ubg = addStance(g, lbg, ubg, ZMP, wft_r_vert, wft_l_vert, Xk, Lk, Rk, k, 'L')


        else: #double stance
            g, lbg, ubg = addStance(g, lbg, ubg, ZMP, wft_r_vert, wft_l_vert, Xk, Lk, Rk, k, 'D')


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
