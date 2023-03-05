#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry

import sys
import numpy as np

class chase_object(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('chase_object')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        # Declare that the chase_object node is subcribing to the /camera/image/compressed topic.
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoints = []
        with open("wayPoints.txt") as file_in:
            for line in file_in:
                self.waypoints.append((float(line.split()[0]),float(line.split()[1])))
        
        self.firstMessage=True
        self.reached_goal=False
        self.waypoint_count=0
        self.Init = True
        self.odom=Odometry()
 

    def waypoint_traversal(self):
        if(self.reached_goal):
            self.reached_goal=False
            self.waypoint_count+=1
            self.simple_velocity_controller(0.0,0.0)
            print(self.waypoints[self.waypoint_count][0],self.waypoints[self.waypoint_count][1])
        
        x=self.waypoints[self.waypoint_count][0]+0.1
        y=self.waypoints[self.waypoint_count][1]
        
        if(self.waypoint_count==0):
            if(abs(self.globalPos_x-x)>0.06):
                self.simple_velocity_controller(0.15,0.0)
                print(self.globalPos_x-x)
            elif(abs(self.globalPos_x-x)<0.1 and ((self.globalAng-1.57)>6.1 or (self.globalAng-1.57)<0)):
                print(self.globalAng-1.57)
                self.simple_velocity_controller(0.0,0.1)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Reached Goal 0!!")   
                self.reached_goal=True 
        if(self.waypoint_count==1):
            # when y is around 1 turn to heading 3.14 move till x is 0.2, turn till angle less than 1.5 go till y is less than 0.5 
            # turn till angle becomes negative go til x becomes gt 0 turn till angle is gt 1.57 
            # go till y becomes greater than 0
            if(abs(self.globalPos_y-y)>0.01):
                self.simple_velocity_controller(0.15,0.0)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Reached Goal 1!!")  
            print("x",self.globalPos_x-x)
            print("y",self.globalPos_y-y) 
            print("ang",self.globalAng)


    def simple_velocity_controller(self,error_x,error_angular):
        velocity_msg = Twist()
        velocity_msg.angular.z = error_angular
        velocity_msg.linear.x = error_x
        self.velocity_publisher.publish(velocity_msg)
        

    def odom_callback(self, msg):
        self.update_Odometry(msg)
        # self.odom.pose.pose.position.x = msg.pose.pose.position.x-self.initial_odom.pose.pose.position.x
        # self.odom.pose.pose.position.y = msg.pose.pose.position.x-self.initial_odom.pose.pose.position.y
        # self.odom.pose.pose.position.z = self.euler_from_quaternion(msg.pose.pose.orientation)[2]#-self.euler_from_quaternion(self.initial_odom.pose.pose.orientation)[2]
        #print("yaw",self.euler_from_quaternion(msg.pose.pose.orientation)[2])
        self.waypoint_traversal()

    def update_Odometry(self,Odom):
            position = Odom.pose.pose.position
            #Orientation uses the quaternion aprametrization.
            #To get the angular position along the z-axis, the following equation is required.
            q = Odom.pose.pose.orientation
            orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            if self.Init:
                #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
                self.Init = False
                self.Init_ang = orientation
                self.globalAng = self.Init_ang
                Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
                self.Init_pos_x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
                self.Init_pos_y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
                self.Init_pos_z = position.z
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            #We subtract the initial values
            self.globalPos_x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos_x
            self.globalPos_y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos_y
            self.globalAng = orientation - self.Init_ang

    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rclpy.init() #init routine needed for ROS2.
    point_subscriber = chase_object() #Create class object to be used.
    rclpy.spin(point_subscriber) # Trigger callback processing.		

    #Clean up and shutdown.
    point_subscriber.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
    main()
