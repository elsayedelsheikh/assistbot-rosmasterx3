#!/usr/bin/env python3
import serial
import time
import math

class ArduinoComm:
    def __init__(self):
        ## Get parameters
        serial_port = "/dev/ttyUSB0"
        baud_rate = 57600
        self.serial_timeout = 0.2
        self.encoder_resolution = 1024
        self.arduino_loop_rate = 30

        ## Set up serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=self.serial_timeout)
        print("Serial port %s opened" %(serial_port))
        
        while not self.ser.is_open:
            self.ser.open()

    def __del__(self):
        print("Closing serial port")
        if self.ser.is_open:
            self.ser.close()

    def control_cycle(self):
        print("Starting control cycle")
        while True:
            try:
                speed1 = 100
                speed2 = 100
                ## Send Velocity Command
                command = f"m {int(speed1)} {int(speed2)}\r\n" ## Carriage return
                self.ser.write(command.encode())
                time.sleep(1/30)
            except:
                print("Error sending velocity command")
                break


if __name__ == '__main__':
    motor_controller = ArduinoComm()
    motor_controller.control_cycle()
