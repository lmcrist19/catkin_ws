import random

print('Problem 07: Search random number')

done = False
count = 0
minNum = 0
maxNum = int(input('Input the maximum number in guess range: '))
num = random.randint(minNum, maxNum)
print(f'Random value to guess is {num}')

while not done:
    guess = int((maxNum + minNum)/2)
    print(f'Guessing {guess}')
    count += 1
    if guess < num:
        print('Too low')
        minNum = guess
    elif guess > num:
        print('Too high')
        maxNum = guess
    elif guess == num:
        print(f'Correct number {num} in {count} guesses')
        done = True
