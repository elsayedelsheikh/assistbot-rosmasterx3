#!/usr/bin/env python3
import numpy as np
import time

class Kinematics:
    def __init__(self):
        ## Robot Parameters
        self.length = 0.2  # Distance from the front wheels to the rear wheels
        self.width  = 0.2  # Distance from the left wheels to the right wheels
        self.radius = 0.05 # Radius of the wheels

        ## Variables
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.heading = 0.0
        self.last_time = time.time()

    def get_wheel_speeds(self, v_x, v_y, omega):
        """
        Given the desired linear and angular velocities of the robot,
        return the wheel speeds required to achieve this motion.
        
        u [radians/s] = [w1, w2, w3, w4] = H * v [m/s]
        Where:
            H  : Kinematic matrix
            v  : Twist vector [v_x, v_y, omega]
            w1 : Front left wheel
            w2 : Front right wheel
            w3 : Rear right wheel   
            w4 : Rear left wheel

        return: u [radians/s]
        """
        ## Robot kinematics
        l = self.length /2
        w = self.width /2
        r = self.radius

        ## Calculate the wheel speeds
        # Wheel speeds are given by the equation: u = H * v
        V = np.array([v_x, v_y, omega])
        H = np.array(
            [
                [1, -1, -l-w],
                [1,  1,  l+w],
                [1, -1,  l+w],
                [1,  1, -l-w],
            ]
        )
        U = (1/r) * np.dot(H, V)
        return U
    
    def get_odom(self, v_x, v_y, omega):
        """
        Get the robot's odometry given the desired linear and angular velocities.
        return: x, y, heading
        """
        ## Change in Pose(P): P_dot = T * v
        V = np.array([v_x, v_y, omega])
        T = np.array(
            [
                [np.cos(self.heading), -np.sin(self.heading),  0],
                [np.sin(self.heading),  np.cos(self.heading),  0],
                [0                   ,                     0,  1]
            ]
        )
        
        dt = time.time() - self.last_time
        P_dot = np.dot(T, V) * dt

        ## Update the last time
        self.last_time = time.time()
        
        ## Update the robot's pose
        self.pose_x  += P_dot[0]
        self.pose_y  += P_dot[1]
        self.heading += P_dot[2]

    def control_cycle(self):
        print("Starting control cycle")
        while True:
            ## Move
            v_x = 0.1
            v_y = 0.0
            omega = 0.0

            ##TODO: Set the wheel speeds
            wheel_speeds = self.get_wheel_speeds(v_x, v_y, omega)
            print("Wheel Velocities: {}".format(wheel_speeds))

            ##TODO:: Publish the robot's odometry
            self.get_odom(v_x, v_y, omega)
            print("Pose: ({:.2f}, {:.2f})".format(self.pose_x, self.pose_y))
            print("Heading: {}".format(np.degrees(self.heading)))

            time.sleep(1)




if __name__ == '__main__':
    k = Kinematics()
    k.control_cycle()
