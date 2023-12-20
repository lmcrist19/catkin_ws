import random

print('Problem 08 - Create random list, print list, minimum and maximum value')

rand_int1 = random.randint(0,9)
rand_int2 = random.randint(0,9)
rand_int3 = random.randint(0,9)
lst = [rand_int1, rand_int2, rand_int3]
print(f'List: {lst}')
mn = min(lst)
mx = max(lst)
print(f'Minimum Value = {mn}, Maximum Value = {mx}')
