import random

print('Problem 06: Guessing game')

done = False
mx = int(input('Input the maximum number in guess range: '))
rand = random.randint(1,mx)
count = 0

while not done:
    num = int(input('Guess a number: '))
    if num == rand:
        count += 1
        done = True 
    elif num < rand:
        print('Too low')
        count += 1
    elif num > rand:  
        print('Too high')
        count += 1
print(f'Correct number {num} in {count} guesses')
