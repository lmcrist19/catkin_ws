import random

print('Problem 05: Guess number 1-10')

done = False
rand = random.randint(1, 10)

while not done:
    num = int(input('Guess number from (1-10): '))
    if num == rand:
        done = True    
print(f'Correct number {num}')
