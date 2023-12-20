print("Problem 05 - Sort two unique integers")
nm1 = int(input("Enter 1st number: "))
nm2 = int(input("Enter 2nd number: "))
if nm1 > nm2:
    lrg = nm1
    sml = nm2
elif nm2 > nm1:
    lrg = nm2
    sml = nm1
print(f'Numbers from smallest to largest: {sml}, {lrg}')
