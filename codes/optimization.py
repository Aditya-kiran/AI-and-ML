import numpy as np
# from skopt import * 

# def f(x):
#     return (np.sin(5 * x[0]) * (1 - np.tanh(x[0] ** 2)) *
#             np.random.randn() * 0.1)

# res = gp_minimize(f, [(-2.0, 2.0)])
# print(res)

import numpy as np
import numdifftools as nd
from robot import Acrobot
from copy import copy

# def f(x):
#     return x[0]**2 + x[1]**2

# f_jacob = nd.Jacobian(f)

# print(f_jacob([1,2]))

def CostFunction(u, xk, u0, xref, robot):
    # Cost function taken from MatLab's nMPC example code 
    Q = np.diag([100.,10.,0.1, 0.1])
    R = 0.01
    J = 0.

    for ct in range(len(u)):
        uk = u[ct]

        xk1 = IntegrationEstimation(xk, uk, robot);

        # for i in range(len(xk1)):
        J += np.matmul(np.matmul((xk1-xref).T,Q),(xk1-xref))
 
        if ct==0:
            J += (uk-u0)*R*(uk-u0)
        else:
            J += (uk-u[ct-1])*R*(uk-u[ct-1])
        
        xk = copy(xk1)

    return J


params = {}
params['m1'] = 1.
params['m2'] = 1.
params['l1'] = 0.5
params['l2'] = 0.5 
params['g'] = -9.81
params['I1'] = 0.1
params['I2'] = 0.1



params['x0'] = np.array([0, 0, 0, 0]).reshape((4,1)) # current state space
params['Ts'] = 0.05             # time iteration
params['N'] = 10              # Event horizon = 10
params['u0'] = 0               # initial force
params['Duration'] = 2.5        # max time 
params['numIterations'] = int(params['Duration']/params['Ts'])
print('total number of iterations are: {}'.format(params['numIterations']))

# xref1 = np.array([-pi/2, 0, 0, 0]).reshape((4,1)) # reference state space
# xf = np.array([pi, 0, 0, 0]).reshape((4,1))

params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -1.0845, -1.0845, 1.3254, 2.1820, 2.6058, 3.1416], 
                        [0, -1.2763, -2.2228, -2.0331, 0.0265, 2.4347, 4.1818, 4.4895, 3.9700, 3.1416],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

# params['xMidPoints'] = np.array([[0, 0.7022, 0.5790, 0.0513, -2., -2., 1.3254, 2.1820, 2.6058, 3.1416], 
#                         [0, -1.2763, -2.2228, -2.0331, 0.0265, 0.0265, 4.1818, 4.4895, 3.9700, 3.1416],
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
#                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

params['numsegment'] = params['xMidPoints'].shape[1] - 1 # subtracting one to get segments

uopt = np.zeros(params['N']) 
u0 = params['u0']
x = params['x0']
uHistory = [u0]             # force history
xHistory = [params['x0'][:,0].tolist()]       # position history
xRefHistory = []

LB = -100            # input force Lower Bound 
UP = 100             # input force Upper Bound

xref = params['x0']

robot = Acrobot(params)
bnds = ((LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP), (LB, UP))
# index = int(params['numsegment']*ct*params['Ts']/params['Duration']) + 1 
xref[1,0] -= xref[0,0]
xref[3,0] -= xref[2,0]
xRefHistory += [xref[:,0].tolist()]
# print("reference value: {}, {}".format(index, xref.T))


m = CostFunction(u, xk,u, xk, u0, xref, robot)
print(m)
# optimize
# res = minimize(CostFunction, uopt, args=(x, u0, xref, robot), method='SLSQP', jac = jac, bounds=bnds, constraints=cons)

