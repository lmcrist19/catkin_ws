import cmath

print("Problem 04 - Polynomial roots")
print("Find the roots for ax^2 + bx + c = 0")
a = float(input("Enter a: "))
b = float(input("Enter b: "))
c = float(input("Enter c: "))
r1 = (-b + cmath.sqrt(b**2-4*a*c))/2*a
r2 = (-b - cmath.sqrt(b**2-4*a*c))/2*a
print("Root 1 = " + str(r1) + ", Root 2 = " + str(r2))
