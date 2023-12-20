import random

print('Problem 04: Random choices')

done = False
lst = ['apple','orange','banana','berry','kiwi']

while not done:
    for i in range(5):
        fruit = random.choice(lst)
        print(f'Random choice {i+1} is {fruit}')
    done = True
