import turtle

print('Problme 08: Draw squares')

color_list = ["red", "blue", "green"]

for color in color_list:
    turtle.pendown()
    turtle.color(color)

    for i in range(4):
        turtle.forward(100)
        turtle.right(90)
    
    turtle.penup()
    turtle.forward(150)  

input('Press enter to continue...')
