#!/usr/bin/env python3
import serial
import time
import math
import rclpy

class ArduinoComm:
    def __init__(self):
        ## Get parameters
        serial_port = "/dev/ttyACM0"
        serial_port2 = "/dev/ttyACM1" ## Front one
        baud_rate = 57600
        serial_timeout = 0.2

        self.encoder_resolution = 2500
        self.arduino_loop_rate = 30

        ## Receive cmd_vel (Twist)


        ## Publish odom and TF with respect to World


        ## Set up serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=serial_timeout)
        self.ser2 = serial.Serial(serial_port2, baud_rate, timeout=serial_timeout)
        
        while not self.ser.is_open:
            self.ser.open()
        while not self.ser2.is_open:
            self.ser.open()

    def __del__(self):
        print("Closing serial ports")
        if self.ser.is_open:
            self.ser.close()
        if self.ser2.is_open:
            self.ser2.close()


    def control_cycle(self):
        print("Starting control cycle")
        while True:
            try:
                speed = self.encoder_resolution/self.arduino_loop_rate  # 1 rev/sec
                ## Send Velocity Command
                command = f"m {int(speed)} {int(speed)}\r\n" ## Carriage return
                self.ser.write(command.encode())
                self.ser2.write(command.encode())

                time.sleep(1/30)
            except:
                print("Error sending velocity command")
                break


if __name__ == '__main__':
    motor_controller = ArduinoComm()
    motor_controller.control_cycle()
