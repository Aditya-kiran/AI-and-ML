import numpy as np
from matplotlib import pyplot as plt
x0 = 0
y0 = 1
x_f = 10
n = 101
delta_x = (x_f-x0)/(n-1)

x = np.linspace(x0,x_f,n)
y = np.zeros([n])
y[0] = y0
for i in range (1,n):
	y[i] = delta_x*(-y[i-1] + np.sin(x[i-1])) + y[i-1]

for i in range(n):
	print(x[i],y[i])

plt.plot(x,y,'o')
plt.xlabel('value of x')
plt.ylabel('value of y')
plt.show()