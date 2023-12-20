print('Problem 10: Capital Vowels')
sentence = input('Enter a string: ') #This is a test of vowel capitals

vowels = 'aeiou'
newSentence = ""

for i in range(len(sentence)):
    if(sentence[i] == 'a' or sentence[i] == 'e' or sentence[i] == 'i' or sentence[i] == 'o' or sentence[i] == 'u'):
        capital = (sentence[i]).upper()
        newSentence += capital
    else:
        newSentence += sentence[i]
        
print(newSentence)
