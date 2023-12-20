print('Problem 03: Number range check')

done = False
num = int(input('Enter number between (10 - 20) : '))

while not done:
    if num >= 10 and num <= 20:
        print('Thank you')
        done = True
    elif num < 10:
        print('Too low')
        num = int(input('Try again, enter number between (10 - 20) : '))
    elif num > 20:
        print('Too high')
        num = int(input('Try again, enter number between (10 - 20) : '))
    
    

