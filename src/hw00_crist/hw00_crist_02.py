import cmath

print("Problem 02 - Compute cylindrical prism volume")
r = int(input("Enter base radius: "))
h = int(input("Enter prism height: "))
v = round(cmath.pi*r**2*h, 3)
print("Volume =", v)
