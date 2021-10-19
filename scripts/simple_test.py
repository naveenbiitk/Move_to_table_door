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


#(robot_translation, robot_orientation) = self.listener1.lookupTransform('map','base_link',rospy.Time(0))
#(robot_roll, robot_pitch, robot_yaw) = euler_from_quaternion(robot_orientation)

#self.move_to_pose({"rotate_mobile_base": angle_})  #angle should be in radian
#self.move_to_pose({ "translate_mobile_base": distance_})  # quantities are in metres
#self.move_to_pose({ "joint_lift": distance_})
#self.move_to_pose({ "wrist_extension": distance_})





            
            
class go_fridge(hm.HelloNode):

	#hm.HelloNode.main(self, 'stretch_test', 'stretch_test', wait_for_first_pointcloud=False)
	
	def __init__(self):
		hm.HelloNode.__init__(self)
		self.rate = 10.0
		self.joint_states = None
		self.joint_states_lock = threading.Lock()
		self.marker_array_lock = threading.Lock()
		self.marker_array = None
		self.delta = 0.2
		#self.listener = tf.TransformListener()
		print("hello")
		#self.initial_configuration()

		
	
		
		#open fridge half way
		robot_c.base.translate_by(0.3)
		robot_c.push_command()

		rospy.sleep(7)

		robot_c.base.rotate_by(-0.436332)
		robot_c.push_command()

		rospy.sleep(5)

		robot_c.base.translate_by(0.50)
		robot_c.push_command()

		rospy.sleep(7)

		#turn so arm is parallel with front of the fridge
		robot_c.base.rotate_by(-1.13446)
		robot_c.push_command()

		rospy.sleep(5)

		#push the door fully open
		#robot_c.base.translate_by(0.50)
		#robot_c.push_command()

		rospy.sleep(7)

		#rotate so front of base is facing front of fridge 
		#robot_c.base.rotate_by(-1.5708)
		#robot_c.push_command()




	#def initial_configuration(self):
		#hm.HelloNode.main(self, 'stretch_test', 'stretch_test', wait_for_first_pointcloud=False)
		#rospy.sleep(2)
		#self.move_to_pose({'wrist_extension': 0.01})
		#rospy.sleep(2)
		#self.move_to_pose({'joint_lift':0.65, 'joint_wrist_yaw':math.pi/2 })
		#rospy.sleep(2)


if __name__ == '__main__':
	try:
		
		parser = ap.ArgumentParser(description='Simple test of Stretch')
		rospy.init_node('stretch_test')
		args, unknown = parser.parse_known_args()
		go_fridge()
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
