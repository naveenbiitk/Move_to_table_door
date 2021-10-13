#!/usr/bin/env python
from __future__ import division, print_function

import math
#import threading
import argparse as ap
import threading

import rospy
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import hello_helpers.hello_misc as hm
#import stretch_gestures
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#for finding coordinates
import stretch_body.robot
robot_c = stretch_body.robot.Robot()
robot_c.startup()




if __name__ == '__main__':
	try:
		parser = ap.ArgumentParser(description='Simple test of Stretch')
		rospy.init_node('stretch_test')
		args, unknown = parser.parse_known_args()

		#Finding the coordinates
		status=robot_c.get_status()

		arm_pose = status['arm']['pos']
		print("arm_pose:", arm_pose)

		base_pose = status['base']['x']
		print("base pose x:", base_pose)

		base_pose = status['base']['y']
		print("base pose y:", base_pose)

		base_pose = status['base']['theta']
		print("base pose theta:", base_pose)

		lift_pose = status['lift']['pos']
		print("lift pose:", lift_pose)


	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
