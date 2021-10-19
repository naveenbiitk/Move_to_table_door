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
#import stretch_body.robot
#robot_c = stretch_body.robot.Robot()
#robot_c.startup()

class Arucofind(hm.HelloNode):

	def __init__(self):
		hm.HelloNode.__init__(self)
		self.rate = 10.0
		self.joint_states = None
		self.joint_states_lock = threading.Lock()
		self.marker_array_lock = threading.Lock()
		self.marker_array = None
		self.delta = 0.2
		self.listener = tf.TransformListener()

	def marker_array_callback(self, marker_array):
		with self.marker_array_lock:
			self.marker_array = marker_array

	def initial_configuration(self):
		rospy.sleep(2)
		self.move_to_pose({'wrist_extension': 0.01})
		rospy.sleep(2)
		self.move_to_pose({'joint_lift':0.65, 'joint_wrist_yaw':math.pi })
		rospy.sleep(2)

	def scan_for_aruco(self):
		#Initial pose
		initial_head_pan = math.pi/2
		initial_head_tilt = -0.5
		pose = {"joint_head_pan": initial_head_pan, "joint_head_tilt":initial_head_tilt}
		self.move_to_pose(pose)

		#Sweeping the head
		delta_pan = math.pi/6
		head_tilt_settings = (-0.7, -0.5, -0.3)
		fridge_aruco_found = False
		sweeping_clockwise = True

		while not fridge_aruco_found:
			current_pan = pose.get("joint_head_pan")
			#print(current_pan)
			
			if current_pan > 2.4:
				sweeping_clockwise = True
			if current_pan < -2.4:
				sweeping_clockwise = False

			#Move to new pose
			if sweeping_clockwise:
				head_pan = current_pan-delta_pan
			else:
				head_pan = current_pan+delta_pan

			pose.update({"joint_head_pan": head_pan})
			self.move_to_pose(pose)

			for tilt in head_tilt_settings:
				pose.update({"joint_head_tilt": tilt})
				self.move_to_pose(pose)
				rospy.sleep(1)

				if self.marker_array is not None:
					for marker in self.marker_array.markers:
						print("Aruco ID:", marker.id)
						if marker.id == 27:
							return

	def check_angle_threshold(self, angle):
		if abs(angle)>math.pi:
			if angle>math.pi:
				final_ang = -(2*math.pi-angle)
			else:
				final_ang = (2*math.pi+angle)
		else:
			final_ang = angle

		return final_ang 


	def align_with_aruco_frame(self):

		rospy.sleep(1)

		(aruco_translation, aruco_orientation) = self.listener.lookupTransform('map','static_fridge',rospy.Time(0))
		(aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion(aruco_orientation)
		
		orient_reach = False
		while not rospy.is_shutdown() and not orient_reach:
			(robot_translation, robot_orientation) = self.listener.lookupTransform('map','base_link',rospy.Time(0))
			(robot_roll, robot_pitch, robot_yaw) = euler_from_quaternion(robot_orientation)

			error_angle = aruco_yaw - math.pi - robot_yaw
			error_angle = self.check_angle_threshold(error_angle)
			error_threshold = 0.017*3 # rad, about 1deg
			print("error_angle:  ", error_angle/math.pi*180)
			
			if abs(error_angle) > error_threshold:
				self.move_to_pose({"rotate_mobile_base": error_angle/3})

			if abs(error_angle) < error_threshold:
				orient_reach = True        

		    
        		print("Robot Angle: ", robot_yaw, " | Target Angle: ", aruco_yaw-math.pi)

			rospy.sleep(2)


        # target_angle = math.pi/2 # this reflects the 90deg rotation between bed and robot X axes
        # xya, time = self.get_robot_floor_pose_xya(floor_frame=alignment_frame)
  
	def move_aruco_marker(self):

		rospy.sleep(1)

		(aruco_translation, aruco_orientation) = self.listener.lookupTransform('map','static_fridge',rospy.Time(0))
		(aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion(aruco_orientation)
		print("aruco_pose",aruco_translation)
		self.move_to_pose({"translate_mobile_base": aruco_translation[0]})
		rospy.sleep(2)
		self.move_to_pose({"rotate_mobile_base": -math.pi/2})
		rospy.sleep(2)
		self.move_to_pose({"translate_mobile_base": -aruco_translation[1]-0.35})
		rospy.sleep(2)
		self.move_to_pose({"rotate_mobile_base": math.pi/2})

		

	def open_the_fridge(self):
		rospy.sleep(2)
		self.move_to_pose({'wrist_extension': 0.15})
		rospy.sleep(2)
		self.move_to_pose({"translate_mobile_base": 0.17})
		rospy.sleep(2)
		self.move_to_pose({"translate_mobile_base": 0.3})
		rospy.sleep(2)
		self.move_to_pose({"rotate_mobile_base": -0.436332})
		rospy.sleep(2)
		self.move_to_pose({"translate_mobile_base": 0.45})
		rospy.sleep(2)
		self.move_to_pose({"rotate_mobile_base": -1.73446})
			
		

	def main(self):
		
		hm.HelloNode.main(self, 'aruco_test', 'aruco_test', wait_for_first_pointcloud=False)
		rate = rospy.Rate(self.rate)
		print(rospy.get_param('/aruco_marker_info'))
		rospy.Subscriber('/aruco/marker_array', MarkerArray, self.marker_array_callback)
		self.initial_configuration()

		self.scan_for_aruco()

		rospy.sleep(1)
		#self.align_with_aruco_frame()
		pose_found = False
		while not rospy.is_shutdown() and not pose_found:
			if self.marker_array is not None:
				for marker in self.marker_array.markers:
					if marker.id == 27:
						(aruco_translation, aruco_orientation) = self.listener.lookupTransform('map','static_fridge',rospy.Time(0))
						(aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion(aruco_orientation)
						pose_found = True

		print("Aruco--------------pose")
		print(aruco_roll/math.pi*180,aruco_pitch/math.pi*180,aruco_yaw/math.pi*180)

		(robot_translation, robot_orientation) = self.listener.lookupTransform('map','base_link',rospy.Time(0))
		(robot_roll, robot_pitch, robot_yaw) = euler_from_quaternion(robot_orientation)
		print(robot_roll/math.pi*180,robot_pitch/math.pi*180,robot_yaw/math.pi*180)

						
			
		self.align_with_aruco_frame()
		self.move_aruco_marker()
		self.open_the_fridge()
		#self.scan_for_aruco()
		# while not rospy.is_shutdown() and not pose_found:
		# 	if self.marker_array is not None:
		# 		for marker in self.marker_array.markers:
		# 			if marker.id == 27:
		# 				(aruco_translation, aruco_orientation) = self.listener.lookupTransform('map','fridge_pointer',rospy.Time(0))
		# 				print("aruco_pose")
		# 				print(aruco_translation)
		# 				(robot_translation, robot_orientation) = self.listener.lookupTransform('map','base_link',rospy.Time(0))
		# 				print("robot_pose")
		# 				print(robot_translation)

		



if __name__ == '__main__':
	try:
		parser = ap.ArgumentParser(description='Aruco test')
		rospy.init_node('aruco_test')
		args, unknown = parser.parse_known_args()


		node1 = Arucofind()
		#node1 = StretchAction()
		node1.main()

	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
