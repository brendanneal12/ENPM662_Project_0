#!/usr/bin/env python

#Importing Libraries
import rclpy
from matplotlib import pyplot as plt
from rclpy.node import Node
from nav_msgs.msg import Odometry


#Defining Vectors Used in Plotting
TimeVector = []
robot_pose_x = []
robot_pose_y = []
robot_pose_z = []
robot_orient_x = []
robot_orient_y = []
robot_orient_z = []


class Odom_Sub(Node): #Define the Pose Subscriber Node

    def __init__(self): #Init Function
        super().__init__("odom_node") #Name the Node
        self.get_logger().info(f'Odometry subscription on.') #Confirm the Node is Spun Properly

        self.subscription = self.create_subscription(Odometry, 'odom', self.callback, 10) #Subscribe to the 'odom' topic
        self.subscription  # Prevent unused variable warning

    def callback(self, msg): #Define the Callback Function
        timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.

        #Append Robot Pose Information to Arrays for Plotting
        TimeVector.append(timestamp) 
        robot_pose_x.append(msg.pose.pose.position.x)
        robot_pose_y.append(msg.pose.pose.position.y)
        robot_pose_z.append( msg.pose.pose.position.z)
        robot_orient_x.append(msg.pose.pose.orientation.x)
        robot_orient_y.append(msg.pose.pose.orientation.y)
        robot_orient_z.append(msg.pose.pose.orientation.z)

        self.get_logger().info(f'Received odometry data at time: {timestamp}') #confirm the Subscriber has recieved data.


def main(args = None): #Main Function Definition
    print('Starting OL Monitoring.') #Confirmation Main has Started
    rclpy.init(args=args)

    odom_subscriber = Odom_Sub() #Create odometry subscriber node.

    try:
        rclpy.spin(odom_subscriber) #Spin the Node
    except KeyboardInterrupt:
        pass

    odom_subscriber.destroy_node()
    rclpy.shutdown()
    #After Ctl. C, Plot the Results.
    print('End of monitoring: prepare for graph!')



    ##-------------------Plotting Results---------------------------##

    figure, (ax1, ax2) = plt.subplots(1,2)

    ax1.set_title('Roll, Pitch, and Yaw Angles vs. Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Orientation (rad)')
    ax1.plot(TimeVector, robot_orient_x, 'r-', label = 'Roll')
    ax1.plot(TimeVector, robot_orient_y, 'b-', label = 'Pitch')
    ax1.plot(TimeVector, robot_orient_z, 'g-', label = 'Yaw')
    ax1.legend()

    ax2.set_title('X, Y, and Z Position vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cartesian Coordinates (m)')
    ax2.plot(TimeVector, robot_pose_x, 'r-', label = 'X Translation')
    ax2.plot(TimeVector, robot_pose_y, 'b-', label = 'Y Translation')
    ax2.plot(TimeVector, robot_pose_z, 'g-', label = 'Z Translation')
    ax2.legend()

    plt.show()



if __name__ == '__main__':
    main()