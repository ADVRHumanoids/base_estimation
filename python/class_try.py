import casadi as cs

class Container:

    def __init__(self, name, f, lbf, ubf):

        self.f = f
        self.lbf = lbf
        self.ubf = ubf

        self.name = name

    def getName(self):
        return self.name

    def getVariable(self):
        return self.f

    def getLowerBounds(self):
        return self.lbf

    def getUpperBounds(self):
        return self.ubf

    def setStateVariable(self, f):
        self.f = f

    def setLowerBounds(self, lbf):
        self.lbf = lbf

    def setUpperBounds(self, ubf):
        self.ubf = ubf

class Constraint(Container):

    def __init__(cls, name, g, lbg, ubg):
        super().__init__(name, g, lbg, ubg)

class StateVariable(Container):

    def __init__(cls, name, w, lbw=None, ubw=None):
        super().__init__(name, w, lbw, ubw)

class Problem:

    def __init__(self):
        self.state_var_container = list()
        self.var_container = list()
        self.cnstr_container = list()

    def createStateVariable(self, name, dim, lbw=None, ubw=None, prev_nodes=None):
        '''
        create a casadi symbolic variable as class StateVariable
        :param name: name of the variable
        :param dim: dimension of the variable
        :param lbw: lower bounds (if None, set all to infinity)
        :param ubw: upper bounds (if None, set all to infinity)
        :param prev_nodes: get also symbolic value of variable for selected previous nodes
        '''

        w = cs.SX.sym(name, dim)

        if lbw is None:
            lbw = [-cs.inf] * w.shape[0]
        if ubw is None:
            ubw = [cs.inf] * w.shape[0]

        # set previous variable if specified
        if prev_nodes is not None:
            if isinstance(prev_nodes, list):
                for node in prev_nodes:
                    prev_var = cs.SX.sym(name + str(node), dim)
                    setattr(Problem, name + str(node), prev_var)
            else:
                cs.SX.sym(name + str(prev_nodes), dim)
                prev_var = cs.SX.sym(name + str(prev_nodes), dim)
                setattr(Problem, name + str(prev_nodes), prev_var)


        temp_var = StateVariable(name, w, lbw, ubw)
        setattr(Problem, temp_var.getName(), temp_var.getVariable())
        self.state_var_container.append(temp_var.getName())

    def setStateVariable(self, name, var, lbw=None, ubw=None):
        # set a casadi symbolic variable as class StateVariable
        assert(isinstance(var, (cs.casadi.SX, cs.casadi.MX)))
        temp_var = StateVariable(name, var, lbw, ubw)
        setattr(Problem, temp_var.getName(), temp_var.getVariable())
        self.state_var_container.append(temp_var.getName())

    def setVariable(self, name, var):

        assert (isinstance(var, (cs.casadi.SX, cs.casadi.MX)))
        setattr(Problem, name, var)
        self.var_container.append(name)

    def getStateVariable(self, name):

        for var in self.var_container:
            if var.getName() == name:
                return var

        return None

    def setConstraint(self, cnstr):

        assert(isinstance(cnstr, Constraint))
        setattr(Problem, cnstr.getName(), cnstr)
        self.cnstr_container.append(cnstr.getName())

if __name__ == '__main__':

    prb = Problem()

    prb.createStateVariable('x', 6, prev_nodes=-1) # how to do for previous nodes?
    prb.createStateVariable('u', 4)
    prb.createStateVariable('z', 2)
    # todo return a variable so that it can be used instead of class attributes
    # todo saving a variable (or a function) inside the problem is probably completely useless if it's not a STATE variable

    state_fun = prb.x[2:6] + prb.u
    fun = prb.u[2:4] +  prb.z **2

    prb.setStateVariable('state_fun', state_fun) # is it ok?
    prb.setVariable('fun', fun)

    print('prb.x:', prb.x)
    print('vars in prb:', vars(prb))

    # what to do? set variable or ?

    print('prb.fun:', prb.fun)