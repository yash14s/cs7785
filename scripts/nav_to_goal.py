#!/usr/bin/env python3
 
#Pranay Mathur & Yash Srivastava
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped
import sys
import numpy as np
import time
class goToGoal(Node):
 
    def __init__(self):
        # Creates the node.
        super().__init__('goToGoal')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        # Declare that the goToGoal node is subcribing to the /camera/image/compressed topic.
        self.odom_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
 
        self.waypoints = []
        self.waypoint_count=0

        #wp_file = "wayPoints.txt"
        wp_file = "waypoints_sim.txt"

        with open(wp_file) as file_in:
            for line in file_in:
                self.waypoints.append((float(line.split()[0]),float(line.split()[1])))
        #self.robot_pose = PoseWithCovariance()
        self.publish_goals()


    def pose_callback(self, msg):
        #self.robot_pose = PoseWithCovarianceStamped()
        print(msg)
        self.robot_pose = msg

        euclidean_dist=((self.robot_pose.pose.position.x-self.waypoints[self.waypoint_count][0])**2+(self.robot_pose.pose.position.y-self.waypoints[self.waypoint_count][1])**2)**0.5
        if(euclidean_dist<0.1):
            print("goal_reached")
            self.waypoint_count+=1
            if(self.waypoint_count!=len(self.waypoints-1)):
                self.publish_goals()
            else:
                print("completed")
                exit(0)
        
 
    def publish_goals(self):
        print("Publish goals")
        goal = PoseStamped()
        t = self.get_clock().now()
        goal.header.stamp=t.to_msg()
        goal.header.frame_id="map"
        goal.pose.position.x=self.waypoints[self.waypoint_count][0]
        goal.pose.position.y=self.waypoints[self.waypoint_count][1]
        goal.pose.position.z=0.0
        goal.pose.orientation.w=1.0
        goal.pose.orientation.x=0.0
        goal.pose.orientation.y=0.0
        goal.pose.orientation.z=0.0
        self.goal_publisher.publish(goal)
        
        
        #else:
            #self.publish_goals()
 
            ## replace this with feedback that you reached, Take x,y from self.robot pose compare with goal x y
 
# 1.17 0.56
# -0.8 0.0
 
# 0.5 0
# 1.0 0.6
# 3.0 0.0
 
def main():
    rclpy.init() #init routine needed for ROS2.
    nav_to_goal_node = goToGoal() #Create class object to be used.
    rclpy.spin(nav_to_goal_node) # Trigger callback processing.		
 
    #Clean up and shutdown.
    nav_to_goal_node.destroy_node()  
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()