#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
# from assistbot_msgs.srv import DriverComm

class SerialServer(Node):
    def __init__(self):
        super().__init__('driver_gpio_server')

        ## Declare parameters
        self.declare_parameter('debug', False)

        ## L298N motor driver pins
        self.declare_parameter('l298n1_in1', 24)
        self.declare_parameter('l298n1_in2', 23)
        self.declare_parameter('l298n1_in3', 22)
        self.declare_parameter('l298n1_in4', 17)
        self.declare_parameter('l298n1_ena', 25)
        self.declare_parameter('l298n1_enb', 27)
        self.declare_parameter('l298n2_in1', 7)
        self.declare_parameter('l298n2_in2', 8)
        self.declare_parameter('l298n2_in3', 11)
        self.declare_parameter('l298n2_in4', 9)
        self.declare_parameter('l298n2_ena', 26)
        self.declare_parameter('l298n2_enb', 10)

        ## Get parameters
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        l298n1_in1 = self.get_parameter('l298n1_in1').get_parameter_value().integer_value
        l298n1_in2 = self.get_parameter('l298n1_in2').get_parameter_value().integer_value
        l298n1_in3 = self.get_parameter('l298n1_in3').get_parameter_value().integer_value
        l298n1_in4 = self.get_parameter('l298n1_in4').get_parameter_value().integer_value
        l298n1_ena = self.get_parameter('l298n1_ena').get_parameter_value().integer_value
        l298n1_enb = self.get_parameter('l298n1_enb').get_parameter_value().integer_value
        l298n2_in1 = self.get_parameter('l298n2_in1').get_parameter_value().integer_value
        l298n2_in2 = self.get_parameter('l298n2_in2').get_parameter_value().integer_value
        l298n2_in3 = self.get_parameter('l298n2_in3').get_parameter_value().integer_value
        l298n2_in4 = self.get_parameter('l298n2_in4').get_parameter_value().integer_value
        l298n2_ena = self.get_parameter('l298n2_ena').get_parameter_value().integer_value
        l298n2_enb = self.get_parameter('l298n2_enb').get_parameter_value().integer_value
        
        ## Setup GPIOs
        GPIO.setmode(GPIO.BCM)
        ### Set gpio pins as output
        GPIO.setup(l298n1_in1,GPIO.OUT)
        GPIO.setup(l298n1_in2,GPIO.OUT)
        GPIO.setup(l298n1_in3,GPIO.OUT)
        GPIO.setup(l298n1_in4,GPIO.OUT)
        GPIO.setup(l298n1_ena,GPIO.OUT)
        GPIO.setup(l298n1_enb,GPIO.OUT)
        GPIO.setup(l298n2_in1,GPIO.OUT)
        GPIO.setup(l298n2_in2,GPIO.OUT)
        GPIO.setup(l298n2_in3,GPIO.OUT)
        GPIO.setup(l298n2_in4,GPIO.OUT)
        GPIO.setup(l298n2_ena,GPIO.OUT)
        GPIO.setup(l298n2_enb,GPIO.OUT)
        ### Set gpio pins to low
        GPIO.setup(l298n1_in1,GPIO.LOW)
        GPIO.setup(l298n1_in2,GPIO.LOW)
        GPIO.setup(l298n1_in3,GPIO.LOW)
        GPIO.setup(l298n1_in4,GPIO.LOW)
        GPIO.setup(l298n1_ena,GPIO.LOW)
        GPIO.setup(l298n1_enb,GPIO.LOW)
        GPIO.setup(l298n2_in1,GPIO.LOW)
        GPIO.setup(l298n2_in2,GPIO.LOW)
        GPIO.setup(l298n2_in3,GPIO.LOW)
        GPIO.setup(l298n2_in4,GPIO.LOW)
        GPIO.setup(l298n2_ena,GPIO.LOW)
        GPIO.setup(l298n2_enb,GPIO.LOW)

        ### Map each motor to its corresponding pins
        motor_br = [l298n1_ena, l298n1_in1, l298n1_in2, GPIO.PWM(l298n1_ena, 1000)]
        motor_bl = [l298n1_enb, l298n1_in3, l298n1_in4, GPIO.PWM(l298n1_enb, 1000)]
        motor_fr = [l298n2_ena, l298n2_in1, l298n2_in2, GPIO.PWM(l298n2_ena, 1000)]
        motor_fl = [l298n2_enb, l298n2_in3, l298n2_in4, GPIO.PWM(l298n2_enb, 1000)]

        motor_br[3].start(0)  
        motor_bl[3].start(0)
        motor_fr[3].start(0)
        motor_fl[3].start(0)

        self.motor_ids ={
            1: motor_fr, 
            2: motor_fl,
            3: motor_br,
            4: motor_bl,
        }

        if self.debug:
            self.get_logger().info("GPIOs set up")

        ## Set up service
        # self.service = self.create_service(DriverComm, 'driver_cmd', self.command_callback)

        ## Variables
        self.last_linear = None
        self.last_angular = None

        # self.motor_control(1,30,0)
        # self.motor_control(2,30,0)
        # self.motor_control(3,30,0)
        # self.motor_control(4,30,0)


    def __del__(self):
        if self.debug:
            print("Closing serial port")
        # if self.ser.is_open:
        #     self.ser.close()

    ## Function to control the motors
    def motor_control(self, motor_id, pwm, forward_dir):
        """
        Controls the motor using the L298N motor driver module connected to the Raspberry Pi GPIOs.
        Parameters:
            - motor_id: 1 to 4 with the corresponding Motors FR,FL,BR,BL respectively
            - pwm: 0 to 1024
            - forward_dir: 1 or 0 with 1 being forward and 0 being backward
        Return:
            None
        """
        if motor_id in self.motor_ids:
            motor = self.motor_ids[motor_id]
            GPIO.output(motor[1],forward_dir)
            GPIO.output(motor[2],not forward_dir)
            motor[3].ChangeDutyCycle(pwm)
        else:
            if self.debug:
                self.get_logger().warn("Motor id %d not found"%(motor_id))

    # def command_callback(self, request, response):
    #     if self.debug:
    #         self.get_logger().info("Command received: %0.2f, %0.2f" %(request.linear, request.angular))

    #     if request.linear != self.last_linear or request.angular != self.last_angular:
    #         ## Send Velocity Command
    #         command = f"mn {request.linear},{request.angular}"
    #         self.ser.write(command.encode())
    #         if self.debug:
    #             self.get_logger().info("Sending velocity command")
            
    #         while (self.ser.inWaiting() == 0):
    #             if self.debug:
    #                 self.get_logger().info("Waiting for velocity response")
    #             time.sleep(self.serial_timeout/4)
    #         if self.ser.inWaiting() > 0:
    #             _ = self.ser.read(self.ser.inWaiting())

    #         self.last_linear  = request.linear
    #         self.last_angular = request.angular

    #     ## Read response
    #     command = "qo"
    #     self.ser.write(command.encode())
    #     if self.debug:
    #         self.get_logger().info("Requesting sensor data")

    #     while (self.ser.inWaiting() == 0):
    #         if self.debug:
    #             self.get_logger().info("Waiting for sensors response")
    #         time.sleep(self.serial_timeout/4)

    #     if self.ser.inWaiting() > 0:
    #         ser_resp = self.ser.readline().decode().strip()
    #         if ser_resp.startswith("done qo"):
    #             response.success = True
    #             if self.debug:
    #                 self.get_logger().info('Response done qo recieved\n')
    #         else:
    #             if self.debug:
    #                 self.get_logger().warn('Response %s'%(ser_resp))
    #             response.success = False

    #     if response.success:
    #         ser_resp = ser_resp.split(" ")[2].split(",")
    #         response.quaternion = [float(f)/1000 for f in ser_resp[:4]]
    #         response.encoders = [int(d) for d in ser_resp[4:]]
    #     else:
    #         self.get_logger().warn("Failed to get sensor data\n")

    #     return response


def main():
    rclpy.init()

    server = SerialServer()

    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()