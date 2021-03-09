# /usr/bin/env python3
import casadi as cs
import numpy as np

import time
import sympy as sp
import matplotlib.pyplot as plt


def RK4(M, L, x, u, xdot, dt):
    """RK4 Runge-Kutta 4 integrator
    TODO: PUT IT IN ANOTHER CLASS
    Input:
        L: objective fuction to integrate
        M: RK steps
        T: final time
        N: numbr of shooting nodes:
        x: state varibales
        u: controls
    """

    f = cs.Function('f', [x, u], [xdot, L])
    X0 = cs.SX.sym('X0', x.size()[0])
    U = cs.SX.sym('U', u.size()[0])
    X = X0
    Q = 0

    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + dt / 2 * k1, U)
        k3, k3_q = f(X + dt / 2 * k2, U)
        k4, k4_q = f(X + dt * k3, U)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q = Q + dt / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

    return cs.Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])


class Problem:

    def __init__(self, N):

        self.N = N
        self.x = list()
        self.u = list()
        self.lbw = list()
        self.ubw = list()

        self.sym_c = cs.SX

        self.ct = Constraint()

        self.var_opt = dict()
        self.fun_opt = dict()

        self.var_dim = dict()

    def setVariable(self, name, var_dim):

        self.var_opt[name] = list()
        for k in range(self.N):
            self.var_opt[name].append(self.sym_c.sym(name + '_' + str(k), var_dim))
            self.var_dim[name] = var_dim

        self.ct.update(self.var_opt)

    def setFunction(self, name, f):

        self.fun_opt[name] = f

    def getVariable(self, k=[]):
        if k:
            for elem in self.var_opt:
                self.var_opt[elem + '-' + str(k)] = list()
        else:
            pass

        return self.var_opt

    def getFunction(self):

        return self.fun_opt

    def buildProblem(self):

        # w = cs.vertcat(*self.var_opt['x'], *self.var_opt['u'])

        self.lbw += [-cs.inf] * self.var_dim['x'] * self.N + [-cs.inf] * self.var_dim['u'] * self.N
        self.ubw += [cs.inf] * self.var_dim['x'] * self.N + [cs.inf] * self.var_dim['u'] * self.N

        # return w

    def solveProblem(self, w, g):

        w0 = list()
        w0 += list(np.zeros((self.var_dim['x'] + self.var_dim['u']) * self.N))
        #

        print('w:', w)
        print('g:', g)

        J = 1

        prob = {'f': J, 'x': w, 'g': g}
        solver = cs.nlpsol('solver', 'ipopt', prob,
                           {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 0, 'sb': 'yes'},
                            'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

        # Solve the NLP
        sol = solver(x0=w0, lbx=self.lbw, ubx=self.ubw, lbg=self.ct.lbg, ubg=self.ct.ubg)

        w_opt = sol['x'].full().flatten()

        # print(w_opt)


class Constraint:

    def __init__(self):
        self.g = list()
        self.lbg = list()
        self.ubg = list()

        self.g_dict = dict()

    def setConstraintFunction(self, name, g):
        flat_list = [item for sublist in list(self.var_opt.values()) for item in sublist]
        f = cs.Function(name, flat_list, [g])
        self.g_dict[name] = f

        return f

    def addConstraint(self, name):
        f = self.g_dict[name]
        g = f(*list(self.var_opt.values()))
        print(g)
        self.g.append(g)

        return g

    def setConstraintBounds(self, lbg, ubg, nodes):
        self.lbg[nodes[0]:nodes[1]] = [lbg] * (nodes[1] - nodes[0])
        self.ubg[nodes[0]:nodes[1]] = [ubg] * (nodes[1] - nodes[0])

    def setConstraint(self, g):
        self.g.append(g)

    def getConstraints(self):
        return cs.vertcat(*self.g)

    def update(self, var):
        self.var_opt = var


if __name__ == '__main__':
    # N = 2
    # prb = Problem(N)
    # h = 1
    # grav = 9.8
    a = sp.symbols('a')
    b = sp.symbols('b')
    t = sp.symbols('t')
    b = 0.306179839802
    a = -0.0936315503011

    # b = 2
    # a = 0
    fun = a + ((6 * t - 15) * t + 10) * t ** 3 * (b-a)
    dfun = sp.diff(fun, t)

    f1 = sp.plot(fun, (t, 0,1), show=False)
    f2 = sp.plot(dfun, (t, 0,1), show=False)

    print(dfun)
    f1.append(f2[0])
    f1.show()
    # # p = cs.SX.sym('p', 2)  # com position
    # # v = cs.SX.sym('v', 2)  # com velocity
    # # a = cs.SX.sym('a', 2)  # com acceleration
    # # define state variables
    # # x = cs.vertcat(p, v, a)
    #
    # # define control variables
    # # j = cs.SX.sym('j', 2)  # com jerk
    # # u = j  # control
    # # model equation
    # # xdot = cs.vertcat(v, a, j)
    #
    # # integrator = RK4(1, 1, x, u, xdot, 0.01)
    #
    # prb.setVariable('x', 6)
    # prb.setVariable('u', 2)
    #
    #
    # var_opt = prb.getVariable()
    #
    # for k in range(N):
    #     if k > 0:
    #         zmp_old = var_opt['x'][k-1][0:2] - var_opt['x'][k-1][4:6] #* (h / grav)
    #         # prb.ct.setConstraint(zmp_old - var_opt['u'][k])
    #
    #     zmp = var_opt['x'][k][0:2] - var_opt['x'][k][4:6] #* (h / grav)
    #
    #     # prb.ct.setConstraint(var_opt['x'][k][0:2] - var_opt['x'][k][4:6])
    #     # prb.ct.setConstraint(var_opt['u'][k] - var_opt['x'][k][4:6])
    #
    #
    # w = prb.buildProblem()
    # g = prb.ct.getConstraints()
    #
    #
    # w_opt = prb.solveProblem(w, g)
