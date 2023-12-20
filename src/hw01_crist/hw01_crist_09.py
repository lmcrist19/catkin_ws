import turtle

print('Problem 09: Draw polygon')

sides = int(input('Enter the number of sides of the polygon: '))
length = int(input('Enter the side length: '))

for i in range(sides):
    turtle.pendown()
    turtle.forward(length)
    turtle.right(360/sides)
    
input('Press enter to continue...')
