import matplotlib.pyplot as plt
import numpy as np

print('Problem 02: Plot sine and cosine')
x_min = float(input('Enter x_min: '))
x_max = float(input('Enter x_max: '))
dx = float(input('Enter dx: '))

x = np.arange(x_min, x_max, dx)
y1 = np.sin(x)
y2 = np.cos(x)
plt.plot(x, y1, 'k--', label='sin')
plt.plot(x, y2, 'ro-', label='cos')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Sine and Cosine')
plt.legend()
plt.show()
