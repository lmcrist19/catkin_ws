print('Problem 09 - Create a list from 0 to 9. Display the list and the values of the even and the odd indices')
lst = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
odd = lst[1::2]
evn = lst[0::2]
print(f'List: {lst}')
print(f'Even Indices: {evn}')
print(f'Odd Indices: {odd}')
