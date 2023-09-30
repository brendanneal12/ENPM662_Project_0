#!/usr/bin/env python


#Importing Libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry




class CV_Publisher(Node): #Define the Command Velocity Publisher Node

    def __init__(self): #Init Function

        super().__init__('cmd_vel_node') #Naming the Node
        self.get_logger().info(f'Command velocity publisher on.') #Confirmation that the Node is Spun Properly

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) #Create the Publisher

        timer_period = 0.5 #Pub Frequency
        self.timer = self.create_timer(timer_period, self.timer_callback) #Initialize Timer Callback
        self.counter = 0 #Counter Required to Switch from Const. Velocity to 0.

    def timer_callback(self): #Defining the Callback Function
        msg = Twist() #Twist Message Type
        if self.counter < 20: #If the time is less than 10s
            msg.linear.x = 0.1 #Move at a constant velocity of 0.1 m/s
        else: #After 10 Seconds:
            msg.linear.x = 0.0 #Stop

        self.publisher_.publish(msg) #Publish the Message
        self.get_logger().info(f'Published command velocity data.') #Confirm Published
        self.counter += 1 #Increase the Counter



def main(args = None): #Defining the 'Main' Function
    print('Starting OL Control: Move TurtleBot3 1m in 10s.') #Confirmation that Main Functions Properly
    rclpy.init(args=args)

    cmd_vel_pub = CV_Publisher() #Establish the Publisher Node

    try:
        rclpy.spin(cmd_vel_pub) #Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    cmd_vel_pub.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
