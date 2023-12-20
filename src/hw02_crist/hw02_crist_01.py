import numpy as np

print('Problem 01: Vector cross product')

a = np.array([1, 2, 3])
b = np.array([3, -2, 2])
c = np.cross(a,b)
d = np.array([a[1]*b[2]-a[2]*b[1],
     a[2]*b[0]-a[0]*b[2],
     a[0]*b[1]-a[1]*b[0]])

print(f'a = {a}')
print(f'b = {b}')
print(f'a x b = {c}')
print(f'a x b = {d}')
