import time
import RPi.GPIO as GPIO

encoder_pin = 7
GPIO.setmode(GPIO.BOARD)

def callback(encoder_pin):
    print("LOW")

GPIO.setup(encoder_pin, GPIO.IN)
GPIO.add_event_detect(encoder_pin, GPIO.FALLING, callback=callback, bouncetime=300)

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()