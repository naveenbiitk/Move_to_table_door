#!/usr/bin/env python
from __future__ import print_function

#Generic lib
import rospy
import threading

# TF for frame conversion
import tf

# Message lib
import tf2_msgs.msg as tf2_msg
import geometry_msgs.msg as gm_msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class FridgePositionBroadcaster:

	def __init__(self):
		print("ArucoPositionBroadcaster::__init__")


	def build_transform_message(self, position, orientation):

		pose_ = gm_msg.TransformStamped()

		# Header
		pose_.header.frame_id = "map"
		pose_.header.stamp = rospy.Time.now()
		pose_.child_frame_id = "static_fridge"

		# Pose
		pose_.transform.translation.x = position[0]
		pose_.transform.translation.y = position[1]
		pose_.transform.translation.z = position[2]

		pose_.transform.rotation.x = orientation[0]
		pose_.transform.rotation.y = orientation[1]
		pose_.transform.rotation.z = orientation[2]
		pose_.transform.rotation.w = orientation[3]

		return pose_


if __name__ == '__main__':
	node_name = 'fridge_pointer'
	node_id_number = 27
	rospy.init_node('aruco_pose_broadcaster')
	listener = tf.TransformListener()
	publisher_tf = rospy.Publisher("/tf",  tf2_msg.TFMessage, queue_size=1)

	try:
		
		rate = rospy.Rate(10)

		tag_is_found = False

		while not tag_is_found:
			#print("not inside this")
			try:
				(translation, orientation) = listener.lookupTransform('map',node_name,rospy.Time(0))
				print("Aruco tag found! Id no:", node_id_number, "Name:", node_name)
				aruco_position_broadcaster = FridgePositionBroadcaster()
				tag_is_found = True
				print("tag_is_found")
				break
			except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		while not rospy.is_shutdown():
			rospy.sleep(1)
			t = aruco_position_broadcaster.build_transform_message(translation, orientation)
			tf_message = tf2_msg.TFMessage([t])
			publisher_tf.publish(tf_message)
			rate.sleep()
			
			#rospy.spin()
				

	except:
		rospy.loginfo("Interrupt received, shutting down")

