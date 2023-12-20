print('Problem 02: Sum to 50')

done = False
total = 0

while not done:
    num = int(input('Enter number : '))
    total += num
    print(f'The total is ... {total}')
    if total >= 50:
        done = True
        
print('While loop end')
