import RPi.GPIO as GPIO

#front_left
frontleft_a = 4
frontleft_b = 18
#back_right
backright_a = 20
backright_b = 16
frontright_a = 19
frontright_b = 13
backleft_b = 6
backleft_a = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(frontright_a, GPIO.IN)
GPIO.setup(frontright_b, GPIO.IN)
GPIO.setup(frontleft_a, GPIO.IN)
GPIO.setup(frontleft_b, GPIO.IN)
GPIO.setup(backright_a, GPIO.IN)
GPIO.setup(backright_b, GPIO.IN)
GPIO.setup(backleft_a, GPIO.IN)
GPIO.setup(backleft_b , GPIO.IN)

outcome = [0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0]
FLlast_AB = 0b00
FRlast_AB = 0b00
BRlast_AB = 0b00
BLlast_AB = 0b00
FRcounter = 0
FLcounter = 0
BRcounter = 0
BLcounter = 0

while True: 
    FLA = GPIO.input(frontleft_a)
    FLB = GPIO.input(frontleft_b)
    BRA = GPIO.input(backright_a)
    BRB = GPIO.input(backright_b)
    FRA = GPIO.input(frontright_a)
    FRB = GPIO.input(frontright_b)
    BLA = GPIO.input(backleft_a)
    BLB = GPIO.input(backleft_b)
    current_FLAB = (FLA << 1) | FLB
    current_BRAB = (BRA << 1) | BRB
    current_FRAB = (FRA << 1) | FRB
    current_BLAB = (BLA << 1) | BLB
    FL_position = (FLlast_AB << 2) | current_FLAB
    BR_position = (BRlast_AB << 2) | current_BRAB
    FR_position = (FRlast_AB << 2) | current_FRAB
    BL_position = (BLlast_AB << 2) | current_BLAB
    FLcounter += outcome[FL_position]
    BRcounter += outcome[BR_position]
    FRcounter += outcome[FR_position]
    BLcounter += outcome[BL_position]
    FLlast_AB = current_FLAB
    BRlast_AB = current_BRAB
    FRlast_AB = current_FRAB
    BLlast_AB = current_BLAB
    print ("FL = " + str(FLcounter), "FR = " + str(FRcounter), "BL = " + str(BLcounter), "BR = " + str(BRcounter))