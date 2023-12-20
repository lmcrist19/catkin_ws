import matplotlib.pyplot as plt
import numpy as np

print('Problem 03: Subplots')

x = np.arange(0,1.0,0.01)

y1 = x
plt.subplot(2, 2, 1)
plt.plot(x, y1, '-b')
plt.ylabel('y=x')

y2 = 1 - x
plt.subplot(2, 2, 2)
plt.plot(x, y2, '-b')
plt.ylabel('y=1-x')

y3 = x**2
plt.subplot(2, 2, 3)
plt.plot(x, y3, '-b')
plt.ylabel('y=$x^2$')

y4 = np.e**x
plt.subplot(2, 2, 4)
plt.plot(x, y4, '-b')
plt.ylabel('y=$e^x$')

plt.show()
