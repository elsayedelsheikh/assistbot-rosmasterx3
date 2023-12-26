#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

from assistbot_base.pid_controller import PIDController 

class LowLevelController(Node):
    def __init__(self):
        super().__init__('low_level_controller')

        ## Declare parameters
        self.declare_parameter('debug', True)

        ## PID parameters
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)

        ## Robot Parameters
        self.declare_parameter('number_of_ticks_per_revolution', 20)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.20)  # Distance between the wheels left to right
        self.declare_parameter('wheel_base', 0.22)       # Distance between the wheels front to back
        # self.declare_parameter('max_linear_speed', 0.5)

        self.TPR = self.get_parameter('number_of_ticks_per_revolution').get_parameter_value().integer_value
        wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        # self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value

        ## Calculate constants for the H(0) matrix
        l = wheel_base / 2
        w = wheel_separation / 2
        
        ## L298N motor driver pins
        self.declare_parameter('PWM_MAX', 80)
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

        ## Encoder pins
        self.declare_parameter('frontright_a', 19)
        self.declare_parameter('frontright_b', 13)
        self.declare_parameter('frontleft_a', 4)
        self.declare_parameter('frontleft_b', 18)
        self.declare_parameter('backright_a', 20)
        self.declare_parameter('backright_b', 16)
        self.declare_parameter('backleft_a', 5)
        self.declare_parameter('backleft_b', 6)

        ## Get parameters
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        ## L298N motor driver pins
        self.PWM_MAX = self.get_parameter('PWM_MAX').get_parameter_value().integer_value
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
        
        ## Encoder pins
        frontright_a = self.get_parameter('frontright_a').get_parameter_value().integer_value
        frontright_b = self.get_parameter('frontright_b').get_parameter_value().integer_value
        frontleft_a = self.get_parameter('frontleft_a').get_parameter_value().integer_value
        frontleft_b = self.get_parameter('frontleft_b').get_parameter_value().integer_value
        backright_a = self.get_parameter('backright_a').get_parameter_value().integer_value
        backright_b = self.get_parameter('backright_b').get_parameter_value().integer_value
        backleft_a = self.get_parameter('backleft_a').get_parameter_value().integer_value
        backleft_b = self.get_parameter('backleft_b').get_parameter_value().integer_value
        
        output_pins = [
            l298n1_in1, l298n1_in2,
            l298n1_in3, l298n1_in4, 
            l298n1_ena, l298n1_enb, 
            l298n2_in1, l298n2_in2, 
            l298n2_in3, l298n2_in4, 
            l298n2_ena, l298n2_enb
        ]

        input_pins = [
            frontleft_a, frontleft_b,
            frontright_a, frontright_b,
            backright_a, backright_b,
            backleft_a, backleft_b
        ]
        
        ## Setup GPIOs
        GPIO.setmode(GPIO.BCM)
        ### Set motor driver gpio pins as output
        for pin in output_pins:
            GPIO.setup(pin,GPIO.OUT)
            GPIO.output(pin,GPIO.LOW)
        ### Set encoder gpio pins as input
        for pin in input_pins:
            GPIO.setup(pin,GPIO.IN)

        ### Setup motor ids for FrontLeft, FrontRight, BackRight, BackLeft respectively
        self.motor_ids ={
            1: [GPIO.PWM(l298n2_enb, 1000), l298n2_in3, l298n2_in4], 
            2: [GPIO.PWM(l298n2_ena, 1000), l298n2_in1, l298n2_in2],
            3: [GPIO.PWM(l298n1_ena, 1000), l298n1_in1, l298n1_in2],
            4: [GPIO.PWM(l298n1_enb, 1000), l298n1_in3, l298n1_in4],
        }

        ### Start PWM
        for motor in self.motor_ids.values():
            motor[0].start(0)

        ### Setup encoder ids for FrontLeft, FrontRight, BackRight, BackLeft respectively
        self.encoder_ids = {
            1: [frontleft_a, frontleft_b],
            2: [frontright_a, frontright_b],
            3: [backright_a, backright_b],
            4: [backleft_a, backleft_b],
        }

        if self.debug:
            self.get_logger().info("GPIO setup complete")

        ### Setup PID controller
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.pid_controllers = [PIDController(self.Kp, self.Ki, self.Kd) for _ in range(4)]

        ## Setup ROS2
        if self.debug:
            self.get_logger().info("Starting Low Level Controller")
        ## Setup timer to run at 60Hz for control cycle
        self.timer = self.create_timer(1/60, self.control_cycle)
        ## Setup subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        ## Variables
        self.twist = np.zeros((3,1)) # Angular.z, Linear.x, Linear.y
        self.h_mtx = np.array([
            [-l-w, 1, -1],
            [l+w , 1,  1],
            [l+w , 1, -1],
            [-l-w, 1,  1]
        ]) / wheel_radius
        self.encoder_counters = [0,0,0,0] # FrontLeft, FrontRight, BackRight, BackLeft respectively

    def __del__(self):
        if self.debug:
            print("Cleaning up GPIO")
        GPIO.cleanup()  
    
    def cmd_vel_callback(self, msg):
        """
        Callback function for cmd_vel topic
        Parameters:
            - msg: Twist message
        Return:
            None
        """
        self.twist = np.array([[msg.angular.z], [msg.linear.x], [msg.linear.y]])
    
    def calc_motor_target_rps(self):
        """
        Calculates the require rps for the motors given linear and angular velocities of the chassis for a mecanum drive
        Return:
            - target_rps: float
            - forward_dir: 1 or -1
        """
        u = self.h_mtx.dot(self.twist) ## u is the velocity of the wheel
        target_rps = u / (2 * np.pi)
        forward_dir = np.sign(target_rps)
        return target_rps, forward_dir

    def control_cycle(self):
        encoder_val = [self.get_encoder(i) for i in range(1,5)]
        self.encoder_counters = [self.encoder_counters[i] + encoder_val[i] for i in range(4)]
        current_rps = [self.encoder_counters[i] / self.TPR for i in range(4)]
        target_rps, forward_dir = self.calc_motor_target_rps()
        error = [target_rps[i] - current_rps[i] for i in range(4)]
        output = [self.pid_controllers[i].update(error[i]) for i in range(4)]
        # for i in range(4):
        #     self.motor_pwm(i+1, output[i], forward_dir[i])
        if self.debug:
            self.get_logger().info(f"Encoder: {encoder_val}")
            self.get_logger().info(f"Encoder Counters: {self.encoder_counters}")
            self.get_logger().info(f"Current RPS: {current_rps}")
            self.get_logger().info(f"Target RPS: {target_rps}")
            self.get_logger().info(f"Error: {error}")
            self.get_logger().info(f"PWM: {output}")
            self.get_logger().info(f"Forward Direction: {forward_dir}")
        
    def get_encoder(self, encoder_id):
        """
        Read encoder value
        Parameters:
            - encoder_id: 1 to 4 with the corresponding Encoders FrontLeft, FrontRight, BackRight, BackLeft respectively
        Return:
            - encoder_value: int
        """
        encoder = self.encoder_ids[encoder_id]
        encoder_a = GPIO.input(encoder[0])
        encoder_b = GPIO.input(encoder[1])
        encoder_value = (encoder_a << 1) | encoder_b
        return encoder_value
        
    ## Function to control the motors
    def motor_pwm(self, motor_id, pwm, forward_dir):
        """
        Controls the motor using the L298N motor driver module connected to the Raspberry Pi GPIOs.
        Parameters:
            - motor_id: 1 to 4 with the corresponding Motors FrontLeft, FrontRight, BackRight, BackLeft respectively
            - pwm: 0 to PWM_MAX
            - forward_dir: 1 or 0 with 1 being forward and 0 being backward
        Return:
            None
        """
        if pwm > self.PWM_MAX:
            pwm = self.PWM_MAX
        motor = self.motor_ids[motor_id]
        GPIO.output(motor[1],forward_dir)
        GPIO.output(motor[2],not forward_dir)
        motor[0].ChangeDutyCycle(pwm)

def main():
    rclpy.init()

    controller = LowLevelController()

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()