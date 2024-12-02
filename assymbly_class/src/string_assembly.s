; The goal for this assmebly code is to detect whether a letter is in a string
; Pass in:  = R25;24 address of the string, R22 = letter to search for (char)
; Return: 1 if the letter is in the string, 0 otherwise

.global findLetter ; Make the function findLetter global and accessible from other files (use in the C file)

findLetter: ; A label
    movw r26, r24 ; Copy the address of the string to R26:27 from R25:24
again:
    ld r18, x+ ; Load the letter to search for into R18
    cpi r18, 0 ; Check if the letter is the null terminator
    breq nomatch ; If it is, the letter is not in the string
    cp r18, r22 ; Compare the letter to search for with the letter in the string
    brne again ; If they are not equal, go back to the beginning of the loop
    ldi r24, 1 ; If the loop finishes, the letter is in the string, so return 1
    ret ; If they are equal, return 1
nomatch:
    ldi r24, 0 ; If the loop finishes, the letter is not in the string, so return 0
    ret ; If they are not equal, return 0