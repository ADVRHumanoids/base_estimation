#/usr/bin/env python2.7

from casadi import *
from numpy import *


N = 100; # number of control intervals
T = 2.0; # time horizon

grav = 9.81
h = 1.

# state variables
p = MX.sym('p',2) # com position
v = MX.sym('v',2) # com velocity
x = vertcat(p, v) # state

# control variables
a = MX.sym('a',2)# com acceleration
u = a # control

# model equation
xdot = vertcat(v, a)

# Objective terms
L = vertcat(x,u)**2

# Formulate discrete time dynamics
# Fixed step Runge-Kutta 4 integrator
M = 1 # RK4 steps per interval
DT = T / N / M
f = Function('f', [x, u], [xdot, L])

X0 = MX.sym('X0', 4)
U = MX.sym('U', 2)
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
for k in range(N):
    # STATE
    Xk = MX.sym('X_' + str(k), 4)
    w += [Xk]
    if (k == 0):
        lbw += [0., 0., 0., 0.]
        ubw += [0., 0., 0., 0.]
    else:
        lbw += [-100., -100., -100., -100.]
        ubw += [100., 100., 100., 100.]

    w0 += [0, 0, 0, 0]

    # CONTROL
    Uk = MX.sym('U_' + str(k), 2)
    w += [Uk]
    if (k == 0):
        lbw += [0., 0.]
        ubw += [0., 0.]
    elif (k == N-1):
        lbw += [0., 0.]
        ubw += [0., 0.]
    else:
        lbw += [-1000., -1000.]
        ubw += [1000., 1000.]

    w0 += [0, 0]

    # Integrator
    Fk = F(x0=Xk, p=Uk)
    XInt += [Fk['xf']]
    J = J + mtimes(Fk['qf'].T, Fk['qf'])

    # Multiple Shooting
    if(k > 0):
        g += [XInt[k-1] - Xk]
        lbg += [0, 0, 0, 0]
        ubg += [0, 0, 0, 0]

    #ZMP
    ZMP = Xk[0:2] - Uk * (h / grav)
    if k <= 40:
        J += 10 * mtimes(ZMP.T, ZMP)
    elif k > 40 and k <= 60:
        J += 100 * mtimes((ZMP - [0.5, 0.5]).T, (ZMP - [0.5, 0.5]))
    elif k > 60:
        J += 100 * mtimes((ZMP - [1., -0.5]).T, (ZMP - [1., -0.5]))


# FINAL STATE
Xk = MX.sym('X_' + str(N), 4)
w += [Xk]
lbw += [-100., -100., 0., 0.]
ubw += [100., 100., 0., 0.]

w0 += [0, 0, 0, 0]

g += [XInt[N-1] - Xk]
lbg += [0, 0, 0, 0]
ubg += [0, 0, 0, 0]


# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob);

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()


# Plot the solution
p_opt = []
v_opt = []
a_opt = []

for k in range(N):
    sol_k = w_opt[6*k:(6*k+6)]
    p_opt = p_opt + list(sol_k[0:2])
    v_opt = v_opt + list(sol_k[2:4])
    a_opt = a_opt + list(sol_k[4:6])



sol_k = w_opt[-4:]
p_opt = p_opt + list(sol_k[0:2])
v_opt = v_opt + list(sol_k[2:4])
#
tgrid = [T/N*k for k in range(N+1)]
px = p_opt[0::2]
vx = v_opt[0::2]
ax = a_opt[0::2]
zmpx = list(np.array(px[0:-1]) - np.array(ax)*(h/grav))
ax.insert(0,float('nan'))
zmpx.append(float('nan'))
#

import matplotlib.pyplot as plt
plt.figure(1)
plt.clf()
plt.plot(tgrid, px, '--')
plt.plot(tgrid, vx, '-')
plt.step(tgrid, ax, '-.')
plt.plot(tgrid, zmpx, '--')
plt.xlabel('t')
plt.legend(['px','vx','ux','zmpx'])
plt.grid()

py = p_opt[1::2]
vy = v_opt[1::2]
ay = a_opt[1::2]
zmpy = list(np.array(py[0:-1]) - np.array(ay)*(h/grav))
ay.insert(0,float('nan'))
zmpy.append(float('nan'))

plt.figure(2)
plt.clf()
plt.plot(tgrid, py, '--')
plt.plot(tgrid, vy, '-')
plt.step(tgrid, ay, '-.')
plt.plot(tgrid, zmpy, '--')
plt.xlabel('t')
plt.legend(['py','vy','uy', 'zmpy'])
plt.grid()

plt.figure(3)
plt.clf()
plt.plot(px, py, '-')
plt.plot(zmpx, zmpy, '-')
plt.legend(['p','zmp'])
plt.xlabel('x')
plt.ylabel('y')

print px

plt.show()