import RPi.GPIO as GPIO
import pygame          
from time import sleep
#back
in4 = 17
in3 = 22
in1 = 24
in2 = 23
ena1 = 25
enb1 = 27

#front
in2_1 = 7
in2_2 = 8
in2_3 = 11
in2_4 = 9
ena2 = 26
enb2 = 10
temp1=1

pygame.joystick.init()
pygame.init()
joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
print(joysticks)

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(ena1,GPIO.OUT)
GPIO.setup(enb1,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

GPIO.setup(in2_1,GPIO.OUT)
GPIO.setup(in2_2,GPIO.OUT)
GPIO.setup(in2_3,GPIO.OUT)
GPIO.setup(in2_4,GPIO.OUT)
GPIO.setup(ena2,GPIO.OUT)
GPIO.setup(enb2,GPIO.OUT)
GPIO.output(in2_1,GPIO.LOW)
GPIO.output(in2_2,GPIO.LOW)
GPIO.output(in2_3,GPIO.LOW)
GPIO.output(in2_4,GPIO.LOW)
p=GPIO.PWM(ena1,1000)
q=GPIO.PWM(enb1,1000)
t=GPIO.PWM(ena2,1000)
u=GPIO.PWM(enb2,1000)

q.start(25)
p.start(25)
t.start(25)
u.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forwards.....")
print("Press: Asterisk/A for Forwards, Moon/B for Backwards, Triangular Arrow/Y for Stop.")
print("\n")    

while(1):

    #x = input()
    x = 'a'
    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            
            if x=='r':
                print("run")
                if(temp1==1):
                    GPIO.output(in1,GPIO.HIGH)
                    GPIO.output(in2,GPIO.LOW)
                    GPIO.output(in3,GPIO.HIGH)
                    GPIO.output(in4,GPIO.LOW)
                    GPIO.output(in2_1,GPIO.HIGH)
                    GPIO.output(in2_2,GPIO.LOW)
                    GPIO.output(in2_3,GPIO.HIGH)
                    GPIO.output(in2_4,GPIO.LOW)
                    print("forward")
                    x='z'

                else:
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.HIGH)
                    GPIO.output(in3,GPIO.LOW)
                    GPIO.output(in4,GPIO.HIGH)
                    GPIO.output(in2_1,GPIO.LOW)
                    GPIO.output(in2_2,GPIO.HIGH)
                    GPIO.output(in2_3,GPIO.LOW)
                    GPIO.output(in2_4,GPIO.HIGH)
                    print("backward")
                    x='z'


            elif pygame.joystick.Joystick(0).get_button(3):
                    print("stop")
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.LOW)
                    GPIO.output(in3,GPIO.LOW)
                    GPIO.output(in4,GPIO.LOW)
                    GPIO.output(in2_1,GPIO.LOW)
                    GPIO.output(in2_2,GPIO.LOW)
                    GPIO.output(in2_3,GPIO.LOW)
                    GPIO.output(in2_4,GPIO.LOW)
                    x='z'

            elif pygame.joystick.Joystick(0).get_button(2):
                    print("Right")
                    GPIO.output(in1,GPIO.HIGH)
                    GPIO.output(in2,GPIO.LOW)
                    GPIO.output(in3,GPIO.LOW)
                    GPIO.output(in4,GPIO.HIGH)
                    GPIO.output(in2_1,GPIO.LOW)
                    GPIO.output(in2_2,GPIO.HIGH)
                    GPIO.output(in2_3,GPIO.HIGH)
                    GPIO.output(in2_4,GPIO.LOW)
                    temp1=1
                    x='z'

            elif pygame.joystick.Joystick(0).get_button(0):
                    print("forward")
                    GPIO.output(in1,GPIO.HIGH)
                    GPIO.output(in2,GPIO.LOW)
                    GPIO.output(in3,GPIO.HIGH)
                    GPIO.output(in4,GPIO.LOW)
                    GPIO.output(in2_1,GPIO.HIGH)
                    GPIO.output(in2_2,GPIO.LOW)
                    GPIO.output(in2_3,GPIO.HIGH)
                    GPIO.output(in2_4,GPIO.LOW)
                    temp1=1
                    x='z'


            elif pygame.joystick.Joystick(0).get_button(1):
                    print("backward")
                    GPIO.output(in1,GPIO.LOW)
                    GPIO.output(in2,GPIO.HIGH)
                    GPIO.output(in3,GPIO.LOW)
                    GPIO.output(in4,GPIO.HIGH)
                    GPIO.output(in2_1,GPIO.LOW)
                    GPIO.output(in2_2,GPIO.HIGH)
                    GPIO.output(in2_3,GPIO.LOW)
                    GPIO.output(in2_4,GPIO.HIGH)
                    temp1=0
                    x='z'
                    

            elif x=='l':
                    print("low")
                    p.ChangeDutyCycle(25)
                    q.ChangeDutyCycle(25)
                    t.ChangeDutyCycle(25)
                    u.ChangeDutyCycle(25)
                    x='z'

            elif x=='m':
                    print("medium")
                    p.ChangeDutyCycle(50)
                    q.ChangeDutyCycle(50)
                    t.ChangeDutyCycle(50)
                    u.ChangeDutyCycle(50)
                    x='z'

            elif x=='h':
                    print("high")
                    p.ChangeDutyCycle(75)
                    q.ChangeDutyCycle(75)
                    t.ChangeDutyCycle(75)
                    u.ChangeDutyCycle(75)
                    x='z'
                
                
            elif x=='e':
                    GPIO.cleanup()
                    print("GPIO Clean up")
                    break
    