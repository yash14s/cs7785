#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

"""
Basic navigation demo to go to pose.
"""


def main():
	rclpy.init()

	navigator = BasicNavigator()

	# Wait for navigation to fully activate, since autostarting nav2
	navigator.waitUntilNav2Active()

	# If desired, you can change or load the map as well
	# navigator.changeMap('/path/to/map.yaml')

	# You may use the navigator to clear or obtain costmaps
	# navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
	# global_costmap = navigator.getGlobalCostmap()
	# local_costmap = navigator.getLocalCostmap()

	# Get waypoints from text file
	waypoints = []
	with open("wayPoints.txt") as file_in:
		for line in file_in:
			#waypoints.append((float(line.split()[0]),float(line.split()[1])))
			goal_pose = PoseStamped()
			goal_pose.header.frame_id = 'map'
			goal_pose.header.stamp = navigator.get_clock().now().to_msg()
			goal_pose.pose.position.x = float(line.split()[0])
			goal_pose.pose.position.y = float(line.split()[1])
			goal_pose.pose.orientation.w = 1.0
			goal_pose.pose.orientation.x = 0.0
			goal_pose.pose.orientation.y = 0.0
			goal_pose.pose.orientation.z = 0.0
			waypoints.append(goal_pose)

	navigator.followWaypoints(waypoints)

	'''
	for i in range(len(waypoints)):

		# Go to our demos first goal pose
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = navigator.get_clock().now().to_msg()
		goal_pose.pose.position.x = waypoints[i][0]
		goal_pose.pose.position.y = waypoints[i][1]
		goal_pose.pose.orientation.w = 1.0

		# sanity check a valid path exists
		# path = navigator.getPath(initial_pose, goal_pose)

	navigator.goToPose(goal_pose)
	while navigator.getResult() != TaskResult.SUCCEEDED:
		if navigator.getResult() == TaskResult.FAILED or navigator.getResult() == TaskResult.CANCELED:
			break
		time.sleep(0.1)
	'''

	result = navigator.getResult()
	if result == TaskResult.SUCCEEDED:
		print('Goal succeeded!')
	elif result == TaskResult.CANCELED:
		print('Goal was canceled!')
	elif result == TaskResult.FAILED:
		print('Goal failed!')
	else:
		print('Goal has an invalid return status!')

	navigator.lifecycleShutdown()

	exit(0)

if __name__ == '__main__':
	main()