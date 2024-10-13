#!/usr/bin/env python3
import serial
import time
import math

class ArduinoComm:
    def __init__(self):
        ## Get parameters
        serial_port = "/dev/ttyUSB0"
        baud_rate = 115200
        self.serial_timeout = 0.2
        self.encoder_resolution = 1024
        self.arduino_loop_rate = 30

        ## Set up serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=self.serial_timeout)
        if self.debug:
            self.get_logger().info("Serial port %s opened" %(serial_port))
        
        while not self.ser.is_open:
            self.ser.open()

    def __del__(self):
        if self.debug:
            print("Closing serial port")
        if self.ser.is_open:
            self.ser.close()

    def control_cycle(self):
        ## Get user input
        while True:
            m_speeds = input("Enter <spd1> <spd2> in (rad/sec): ")
            m_speeds = m_speeds.split(" ")
            if len(m_speeds) != 2:
                print("Invalid input")
                continue
            try:
                speed1 = float(m_speeds[0]) * self.encoder_resolution / (self.arduino_loop_rate * 2 * math.pi)
                speed2 = float(m_speeds[1]) * self.encoder_resolution / (self.arduino_loop_rate * 2 * math.pi)
                ## Send Velocity Command
                command = f"m {int(speed1)} {int(speed2)}"
                self.ser.write(command.encode())
                print("Sending velocity command")
                
                while (self.ser.inWaiting() == 0):
                    print("Waiting for velocity response")
                    time.sleep(self.serial_timeout/4)
                if self.ser.inWaiting() > 0:
                    _ = self.ser.read(self.ser.inWaiting())
            except:
                print("Error sending velocity command")
                break


if __name__ == '__main__':
    motor_controller = ArduinoComm()
    motor_controller.control_cycle()
