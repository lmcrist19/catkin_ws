print('Problem 01: Count down from 50')

done = False
fifty = 50

num = int(input('Enter a number below 50 : '))

while not done:
    if fifty >= num:
        print(fifty)
        fifty -= 1
    else:
        done = True
