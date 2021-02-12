# /usr/bin/env python3
import casadi as cs
import numpy as np

import time


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

        self.var_opt_prev = dict()
        self.var_opt = dict()
        self.fun_opt = dict()

        self.var_dim = dict()

    def setVariable(self, name, var_dim, k_node=[]):

        if k_node:
            self.var_opt[name + str(k_node)] = self.sym_c.sym(name + str(k_node), var_dim)
            self.var_dim[name + str(k_node)] = var_dim
        else:
            self.var_opt[name] = self.sym_c.sym(name, var_dim)
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

        X = list()
        U = list()

        for k in range(self.N):

            print( '----- node', k, '-----')
            self.updateVariables(k)

            X.append(self.var_opt['x'])  # add state variables at every loop
            U.append(self.var_opt['u'])  # add input variables at every loop

            self.lbw += [-cs.inf] * var_opt['x'].shape[0] + [-cs.inf] * var_opt['u'].shape[0]
            self.ubw += [cs.inf] * var_opt['x'].shape[0] + [cs.inf] * var_opt['u'].shape[0]

            print('adding constraint functions:')
            # add new constraint with changed input
            self.ct.update(self.var_opt)
            for constraint in self.ct.g_dict:
                # add constraint only if in the specified nodes
                for chunk in self.ct.g_dict[constraint]['nodes']:
                    if k in range(chunk[0], chunk[1]):
                        self.ct.addConstraint(constraint)

        print('----- adding bounds: ------')
        self.__addConstraintBounds()
        w = cs.vertcat(*X, *U)
        # w = [None] * (len(X) + len(U))
        # w[0::2] = X
        # w[1::2] = U
        g = self.ct.getConstraints()

        return w, g

    def __addConstraintBounds(self):

        for constraint in self.ct.g_dict:
            for k in list(self.ct.g_dict[constraint]['bounds']['lbg'].keys()):
                self.ct.lbg.extend(self.ct.g_dict[constraint]['bounds']['lbg'][k])

            for k in list(self.ct.g_dict[constraint]['bounds']['ubg'].keys()):
                self.ct.ubg.extend(self.ct.g_dict[constraint]['bounds']['ubg'][k])

            print(constraint,'_lbg:', [(key, value) for key, value in self.ct.g_dict[constraint]['bounds']['lbg'].items()])
            print(constraint,'_ubg:',[(key, value) for key, value in self.ct.g_dict[constraint]['bounds']['ubg'].items()])
            print('\n')

    def solveProblem(self, w, g):

        w0 = list()
        w0 += list(np.zeros((self.var_opt['x'].shape[0] + self.var_opt['u'].shape[0]) * self.N))

        print('================')
        # print('w:', w)
        print('g:', g)
        print('lbg:', self.ct.lbg)
        print('ubg:', self.ct.ubg)
        J = 1

        prob = {'f': J, 'x': w, 'g': g}
        solver = cs.nlpsol('solver', 'ipopt', prob,
                           {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 0, 'sb': 'yes'},
                            'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

        # Solve the NLP
        sol = solver(x0=w0, lbx=self.lbw, ubx=self.ubw, lbg=self.ct.lbg, ubg=self.ct.ubg)

        w_opt = sol['x'].full().flatten()

        # print(w_opt)

    def updateVariables(self, k):

        for elem in self.var_opt:
            if elem.find('-') != -1:
                if k > 0:
                    name = elem[:elem.index('-')]
                    self.var_opt_prev[k - 1] = self.var_opt[name]

        for elem in self.var_opt:
            if elem.find('-') == -1:
                self.var_opt[elem] = self.sym_c.sym(elem + '_' + str(k), self.var_dim[elem])
            else:
                k_prev = int(elem[elem.index('-'):])
                if k+k_prev in self.var_opt_prev:
                    self.var_opt[elem] = self.var_opt_prev[k+k_prev]


class Constraint:

    def __init__(self):
        self.g = list()
        self.lbg = list()
        self.ubg = list()

        self.g_dict = dict()

    def setConstraintFunction(self, name, g, nodes=None, bounds=None):

        # TODO should be a list of lists, since there may be more than one interval with same constraint
        if not nodes:
            nodes = [0, prb.N]

        if not any(isinstance(el, list) for el in nodes):
            nodes = [nodes]

        if len(nodes) > len(set([item for sublist in nodes for item in sublist])):
            raise Exception('ERROR: intersecting lists of nodes.')

        # if nodes of constraints is outside the range of nodes in the problem, trim
        if max(nodes)[0] > prb.N:
            print(
            'WARNING: lists of constraints ( max:', [max(nodes)[0], max(nodes)[1]], ') nodes outside the problem nodes ( max:', prb.N,'). Removing.')
            nodes.remove(max(nodes))

        if max(nodes)[1] > prb.N:
            print('WARNING: lists of constraints( max:', max(nodes)[1], ') nodes outside the problem nodes ( max:', prb.N, '). Trimming.')
            max(nodes)[1] = prb.N

        print(nodes)
        # check if list of nodes make sense


        if not bounds:
            bounds = dict()
            bounds['nodes'] = nodes
            bounds['lbg'] = [-cs.inf] * g.shape[0]
            bounds['ubg'] = [cs.inf] * g.shape[0]

        if not any(isinstance(el, list) for el in bounds['nodes']):
            bounds['nodes'] = [bounds['nodes']]

        if 'lbg' not in bounds:
            bounds['lbg'] = [-cs.inf] * g.shape[0]

        if 'ubg' not in bounds:
            bounds['ubg'] = [cs.inf] * g.shape[0]

        if len(bounds['lbg']) != g.shape[0]:
            raise Exception('Dimension of lower bounds (', len(bounds['lbg']), ') '
                                                                               'does not coincide with the constraint dimension (', g.shape[0], ')')
        if len(bounds['ubg']) != g.shape[0]:
            raise Exception('Dimension of upper bounds (', len(bounds['ubg']), ') '
                                                                               'does not coincide with the constraint dimension (', g.shape[0], ')')

        lbg = dict()
        ubg = dict()

        for chunk in nodes:
            lbg.update((elem,[-cs.inf] * g.shape[0]) for elem in list(range(chunk[0], chunk[1]))) #dict(lbg=[-cs.inf] * g.shape[0], ubg=[cs.inf] * g.shape[0])
            ubg.update((elem,[cs.inf] * g.shape[0]) for elem in list(range(chunk[0], chunk[1])))

        for chunk in bounds['nodes']:
            for i in range(chunk[0], chunk[1]):
                if i < prb.N: # check if bounds['nodes'] are inside problem nodes
                    lbg[i] = bounds['lbg']
                    ubg[i] = bounds['ubg']

        used_var = dict()
        # select from all variables only the variables used by the added constrain function
        for name_var, var in list(self.var_opt.items()):
            if cs.depends_on(g, var):
                used_var[name_var] = var
                # check if variable exists in the full range of nodes
                if name_var.find('-') != -1: # get from 'nodes' the first constrained node
                    if min(nodes)[0] - int(name_var[name_var.index('-') + len('-'):]) < 0:
                        raise Exception('Failed to add constraint: variable', name_var, 'can only be added from node n:', int(name_var[name_var.index('-') + len('-'):]))

        # create function and add it to dictionary of constraint function
        f = cs.Function(name, list(used_var.values()), [g])
        self.g_dict[name] = dict(constraint=f, bounds=dict(lbg=lbg, ubg=ubg), nodes=nodes, var=list(used_var.keys()))

        return f

    def addConstraint(self, name):

        f = self.g_dict[name]['constraint']
        g = f(*[self.var_opt[x] for x in self.g_dict[name]['var']])
        print(g)
        self.g.append(g)

    def setConstraintBounds(self, lbg, ubg, nodes):
        self.lbg[nodes[0]:nodes[1]] = [lbg] * (nodes[1] - nodes[0])
        self.ubg[nodes[0]:nodes[1]] = [ubg] * (nodes[1] - nodes[0])

    def getConstraints(self):
        return cs.vertcat(*self.g)

    def update(self, var):
        self.var_opt = var


if __name__ == '__main__':
    N = 5
    prb = Problem(N)
    h = 1
    grav = 9.8

    # a = cs.SX.sym('a', 2)  # com acceleration
    # define state variables
    # x = cs.vertcat(p, v, a)

    # define control variables
    # j = cs.SX.sym('j', 2)  # com jerk
    # u = j  # control
    # model equation
    # xdot = cs.vertcat(v, a, j)

    # integrator = RK4(1, 1, x, u, xdot, 0.01)

    prb.setVariable('x', 6)
    prb.setVariable('u', 2)
    prb.setVariable('x', 6, -2)

    var_opt = prb.getVariable()

    # print(var_opt)
    # exit()
    zmp_old = var_opt['x-2'][0:2] - var_opt['x-2'][4:6]  # * (h / grav)
    zmp = var_opt['x'][0:2] - var_opt['x'][4:6]  # * (h / grav)

    prb.setFunction('zmp', zmp)
    prb.setFunction('zmp_old', zmp_old)

    # exit()
    # Fk = integrator(x0=var_opt['x-1'][0:6], p=var_opt['u-1'])

    fun_opt = prb.getFunction()

    # print(fun_opt)
    # exit()
    # if k > 0:
    # forward integration
    ## Multiple Shooting (the result of the integrator [XInt[k-1]] must be the equal to the value of the next node)
    # prb.ct.addConstraint('multiple_shooting', Fk['xf'] - var_opt['x'][0:6])
    # define template constraint function
    prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], bounds=dict(nodes=[2,4], ubg=[1, 1], lbg=[-1,-1]))
    prb.ct.setConstraintFunction('another_constraint', var_opt['u'] - var_opt['x'][4:6], nodes=[[0,2], [4,5]])
    prb.ct.setConstraintFunction('zmp_constraint', fun_opt['zmp_old'] - var_opt['u'], [2, prb.N])

    w, g = prb.buildProblem()


    # print(w)

    # x and u should be always present
    # add duration of constraint
    # add lbg and ubg
    # add lbw and ubw

    w_opt = prb.solveProblem(w, g)
