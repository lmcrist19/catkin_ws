import matplotlib.pyplot as plt
import numpy as np

print('Problem 05: Rounding')

y_list = []
int_list = []
trap_list = []

x = list(np.arange(0,5.0,0.5))
for i in range(0,len(x)):
    y = (x[i]**2)-(6*x[i])+4
    sym = ((1.0/3.0)*x[i]*3.0)-(3.0*x[i]**2)+(4.0*x[i])
    y_list.append(y)
    int_list.append(sym)

for n in range(0, len(y_list)):
    trap = 0.5*y_list[n]+((y_list[n+1]-y_list[n])*0.5)/2
    trap_list.append(trap)

print(int_list)
print(trap_list)
print(x)
