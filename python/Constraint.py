#/usr/bin/env python3
import casadi as cs
import numpy as np

class Constraint:

    def __init__(self):

        self.g = list()
        self.lbg = list()
        self.ubg = list()

    def addConstraintFunction(self, g):

        dim_g = (g.shape)
        print dim_g
        self.g.append(g)
        # self.lbg.append(cs.inf)
        # self.ubg.append()

if __name__ == '__main__':

    ct = Constraint()
    x = list()
    x = cs.SX.sym('X_1', 6)

    g = x[0:2] - x[4:6]

    print g
    ct.addConstraintFunction(g)

    J = list()
    w = list()
    g = list()
    w0 = list()

    J = 1
    w = x
    g = ct.g

    lbw = ct.lbw
    ubw = ct.ubw
    lbg = ct.lbg
    ubg = ct.ubg

    prob = prob = {'f': J, 'x': cs.vertcat(*w), 'g': cs.vertcat(*g)}
    solver = cs.nlpsol('solver', 'ipopt', prob,
                    {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 0, 'sb': 'yes'},
                     'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

    # Solve the NLP
    w0 += list(np.zeros(x.size()[0]))
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg,ubg=ubg)

    w_opt = sol['x'].full().flatten()