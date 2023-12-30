#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('HBTask1bController')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_odom_sub = self.create_subscription(Odometry, '/odom', self.odometryCb,10)

        # Declare a Twist message **done**
        # Initialise the required variables to 0 **done**
        self.vel = Twist()
      
        self.flag = 0
        # For maintaining control loop rate.

        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        self.rate = self.create_rate(100)
        self.x_d = 1
        self.y_d = 1
        self.theta_d = 0
        
        # and also Kp values for the P Controller
        # Initialize P controller gains
        self.Kp_x = 0.5
        self.Kp_y = 0.5
        self.Kp_theta = 0.5

        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0

        # Create timer to run the control loop at a defined frequency
        self.create_timer(0.01, self.pub_vel)

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('Service not available, waiting again...')
        self.req = NextGoal.Request() 
        self.index = 0
        self.goal_arr = [0,0,0]
        self.move = True

    def odometryCb(self, Odometry):
        self.hb_x = Odometry.pose.pose.position.x
        self.hb_y = Odometry.pose.pose.position.y
        orientation_quaternion = (
            Odometry.pose.pose.orientation.x,
            Odometry.pose.pose.orientation.y,
            Odometry.pose.pose.orientation.z,
            Odometry.pose.pose.orientation.w
        )
        roll, pitch, yaw = euler_from_quaternion(orientation_quaternion)
        self.hb_theta = yaw  
        self.get_logger().info('Odom: "%s"' % self.hb_x, self.hb_y, self.hb_theta)
  
        
    def send_request(self, index):
        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def find_distance(self):
        distance = math.sqrt((self.x_d-self.hb_x)**2 + (self.y_d-self.hb_y)**2)
        return distance 
    
    def pub_vel(self):
        # Publish the required Twist message here
        # self.cmd_vel_pub.publish(....)
        self.vel.linear.x = 0.0

            # Find error (in x, y and theta) in global frame
            # the /odom topic is giving pose of the robot in global frame
            # the desired pose is declared above and defined by you in global frame
            # therefore calculate error in global frame
            
            
        err_x = self.goal_arr[0] - self.hb_x
        err_y = self.goal_arr[1] - self.hb_y
        err_theta = self.goal_arr[2]-self.hb_theta

        rotation_matrix = np.array([
                [np.cos(self.hb_theta), -np.sin(self.hb_theta)],
                [np.sin(self.hb_theta), np.cos(self.hb_theta)]
            ])

        global_vector = np.array([
                [err_x ],
                [err_y ]
            ]) 

        rotated_vector = np.dot(rotation_matrix, global_vector)
        err_x_b, err_y_b = rotated_vector[0,0], rotated_vector[1,0]
        # (Calculate error in body frame)
        # But for Controller outputs robot velocity in robot_body frame, 
        # i.e. velocity are define is in x, y of the robot frame, 
        # Notice: the direction of z axis says the same in global and body frame
        # therefore the errors will have have to be calculated in body frame.
    
        # This is probably the crux of Task 1, figure this out and rest should be fine.

        # Finally implement a P controller 
        # to react to the error with velocities in x, y and theta.
        
        Vx = self.Kp_x * err_x_b
        Vy = self.Kp_y * err_y_b
        Wz = self.Kp_theta * err_theta

        if(Vx>10):
            Vx = 10
        if(Vy>10):
            Vy = 10
        
        self.vel.linear.x = Vx
        self.vel.linear.y = Vy
        self.vel.angular.z = Wz
        self.cmd_vel_pub.publish(self.vel)
        self.rate.sleep()
        

    def stop_robot(self):
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel) 
        self.move = False
        self.get_logger().info('Stopping Robot:')

    def is_at_goal(self):
        if (self.find_distance() < 0.1):
            self.stop_robot()
            self.wait_at_goal()
            self.get_logger().info(f'Goal Reached: {self.hb_x}{self.hb_y}{self.hb_theta}')
            return True
        else:
            return False

    def wait_at_goal(self):
        self.get_logger().info('Reached Goal')
        time.sleep(1)
        self.get_logger().info('Waiting at Goal')
        time.sleep(1)
        self.get_logger().info('Resuming Control Loop')
        time.sleep(1)



def main(args=None):
    rclpy.init(args=args)
    ebot_controller = HBTask1BController()

    ebot_controller.send_request(ebot_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################

                # Find error (in x, y and theta) in global frame
                # the /odom topic is giving pose of the robot in global frame
                # the desired pose is declared above and defined by you in global frame
                # therefore calculate error in global frame
                ebot_controller.goal_arr = [x_goal,y_goal,theta_goal]
                while not ebot_controller.is_at_goal():
                    rclpy.spin_once(ebot_controller)
                ebot_controller.stop_robot()
                if(ebot_controller.is_at_goal()):
                ############     DO NOT MODIFY THIS       #########
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1 :
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)
                    ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)
    
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
