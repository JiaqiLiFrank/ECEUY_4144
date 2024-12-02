; This is a comment in the assembly file
; As the start, we make a function that takes in two values and returns their sum


.global addAssem ; Make the function addAssem global and accessible from other files (use in the C file)

addAssem: ; A label
; passing in two 16-bit values
; Each register, R24, will hold 8 bits of the 16-bit value,
;      so we need two registers to hold the 16-bit value, 
;      making them low and high registers. 
; They will be in the registers R24:25 and R22:23
; Return the result in R24:25

add r24, r22 ; Add the low bytes
adc r25, r23 ; Add the high bytes with carry

ret ; Return the result