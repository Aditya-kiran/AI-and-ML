from sympy import Symbol, Derivative
from IPython import embed

y= Symbol('y')
x = Symbol('x')

function= x**2 * y**3 + 12*y**4

partialderiv= Derivative(function, y)
partialderiv.doit()

embed()