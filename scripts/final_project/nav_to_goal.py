#!/usr/bin/env python3

#Pranay Mathur & Yash Srivastava

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import sys
import numpy as np
import time
from statistics import mode

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
        self.sign_subscriber = self.create_subscription(Int8, '/predicted_label', self.sign_callback, qos_profile)

        self.waypoints = []
        self.laserscan_subscriber = self.create_subscription(
            LaserScan,'/scan',self.scan_callback,qos_profile)

        self.Init=True
        self.reached_goal=False
        self.odom=Odometry()
        self.average_distance=1.5
        self.state=0
        self.sign=""
        self.sign_buffer=[4,4,4,4,4]

    def scan_callback(self, msg):	
        range_data=LaserScan()
        range_data.ranges=msg.ranges
        self.average_distance=range_data.ranges[0]
        self.average_distance_backward=range_data.ranges[70]

    def sign_callback(self,msg):
        sign=msg.data
        if(sign!=0):
            self.sign_buffer.append(sign)
        if(len(self.sign_buffer)>10):
            self.sign_buffer.pop(0)
        if(sign==0):
            self.sign="wall"
        elif(sign==1):
            self.sign="left"
        elif(sign==2):
            self.sign="right"
        elif(sign==3):
            self.sign="turnaround"
        elif(sign==4):
            self.sign="stop"
        elif(sign==5):
            self.sign="goal"

    def move(self):
        if(self.state==0):
            # Give goal here then switch state
            if(self.sign=="left"):
                self.goalAngle=self.globalAng+1.52
                self.state=2
            elif(self.sign=="right"):
                self.goalAngle=self.globalAng-1.52
                self.state=3
            elif(self.sign=="wall"):
                # Need to write some form of recovery beahviour
                self.state=4
            elif(self.sign=="turnaround"):
                self.goalAngle=self.globalAng+3.12
                self.state=5
            elif(self.sign=="stop"):
                # Use same behaviour for turn around
                self.goalAngle=self.globalAng+3.12
                self.state=6
            elif(self.sign=="goal"):
                self.state=7
            else:
                # move forward if sign is default ""
                self.state=1

        elif(self.state==1):
            #move forward
            self.go_forward()

        elif(self.state==2):
            # move left
            self.turn_left()
        
        elif(self.state==3):
            # move left
            self.turn_right()

        elif(self.state==4):
            # Call some behaviour that can identify a sign somewhere close
            sign=mode(self.sign_buffer)
            if(sign==1):
                self.sign="left"
                self.goalAngle=self.globalAng+1.52
                self.state=2
            elif(sign==2):
                self.sign="right"
                self.goalAngle=self.globalAng-1.52
                self.state=3
            elif(sign==3):
                self.sign="turnaround"
                self.goalAngle=self.globalAng+3.12
                self.state=5
            elif(sign==4):
                self.sign="stop"
                self.goalAngle=self.globalAng+3.12
                self.state=5
            elif(sign==5):
                self.sign="goal"
                self.state=7
            print("Using recovery behaviour, Got new state: ",self.state)
            #self.search_for_sign()

        elif(self.state==5):
            self.turn_around()

        elif(self.state==6):
            self.turn_around()
        
        elif(self.state==7):
            self.reached_goal=True

    def turn_around(self):
        print("turning 180")
        print(self.goalAngle,self.globalAng)
        if(self.goalAngle>3.1):
            if(self.globalAng<self.goalAngle and self.globalAng>0):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                self.state=1
        elif(self.goalAngle<3.1):
            if(self.globalAng<self.goalAngle):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                self.state=1

    def search_for_sign(self):
        print("searching for sign")
        self.simple_velocity_controller(-0.1,0.0)
        time.sleep(1)
        if(self.sign!="wall"):
            self.state=0
        else:
            self.simple_velocity_controller(0.0,0.3)
            time.sleep(2)
            self.simple_velocity_controller(0.0,0.0)
            if(self.sign!="wall"):
                self.state=0
            else:
                self.simple_velocity_controller(0.0,-0.3)
                time.sleep(4)
                self.simple_velocity_controller(0.0,0.0)
                if(self.sign!="wall"):
                    self.state=0
                else:
                    self.simple_velocity_controller(0.0,0.3)
                    time.sleep(2)
                    self.simple_velocity_controller(0.0,0.0)
                    self.state=0
        if(self.sign=="wall"):
            print("Aggressive recovery behavior")
            self.goalAngle=self.globalAng+3.12
            self.state=5
            self.turn_around()

    def go_forward(self):
        print("Distance to goal: ",self.average_distance)
        if(self.average_distance>0.5):
            self.simple_velocity_controller(0.12,0.0)
        else:
            self.simple_velocity_controller(0.0,0.0)
            self.state=0

    def turn_left(self):
        print(self.goalAngle,self.globalAng)
        if(self.goalAngle>1.40 and self.goalAngle<1.70):
            if(self.globalAng<self.goalAngle):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned left by 90 deg\n")
                self.state=1
        elif(self.goalAngle>3.0):
            if(self.globalAng<self.goalAngle and self.globalAng>0):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned left by 90 deg\n")
                self.state=1                
        elif(self.goalAngle<-1.4 and self.goalAngle>-1.7):
            if(self.globalAng<self.goalAngle):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned left by 90 deg\n")
                self.state=1
        elif(self.goalAngle>-0.2 and self.goalAngle<0.2):
            if(self.globalAng<self.goalAngle and self.globalAng<0):
                self.simple_velocity_controller(0.0,0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned left by 90 deg\n")
                self.state=1

    def turn_right(self):
        print(self.goalAngle,self.globalAng)
        if(self.goalAngle>1.40 and self.goalAngle<1.70):
            if(self.globalAng>self.goalAngle):
                self.simple_velocity_controller(0.0,-0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned right by 90 deg\n")
                self.state=1
        elif(self.goalAngle>-1.7 and self.goalAngle<-1.4):
            if(self.globalAng>self.goalAngle):
                self.simple_velocity_controller(0.0,-0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned right by 90 deg\n")
                self.state=1                
        elif(self.goalAngle<-3.0 or self.goalAngle>3.1):
            if(self.globalAng>self.goalAngle and self.globalAng<0):
                self.simple_velocity_controller(0.0,-0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned right by 90 deg\n")
                self.state=1
        elif(self.goalAngle>-0.2 and self.goalAngle<0.2):
            if(self.globalAng>self.goalAngle and self.globalAng>0):
                self.simple_velocity_controller(0.0,-0.3)
            else:
                self.simple_velocity_controller(0.0,0.0)
                print("Turned right by 90 deg\n")
                self.state=1

    def go_backward(self):
        # print(self.average_distance)
        if(self.average_distance_backward>0.5):
            self.simple_velocity_controller(0.12,0.0)
        else:
            self.simple_velocity_controller(0.0,0.0)

    def simple_velocity_controller(self,speed_x,speed_angular):
        velocity_msg = Twist()
        velocity_msg.angular.z = speed_angular
        velocity_msg.linear.x = speed_x
        self.velocity_publisher.publish(velocity_msg)
        

    def odom_callback(self, msg):
        self.update_Odometry(msg)
        #self.go_forward()
        if(self.reached_goal):
            print("Goal Reached!!")
        else:
            self.move()

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
            self.globalAng = np.arctan2(np.sin(self.globalAng), np.cos(self.globalAng))


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
