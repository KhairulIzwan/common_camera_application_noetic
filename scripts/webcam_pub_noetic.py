#!/usr/bin/env python3

################################################################################
## {Description}: Accessing raspicam/usbcam
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy


class WebcamPublish:

	def __init__(self):
	
		self.bridge = CvBridge()
		self.image_received = False

		rospy.logwarn("WebcamPublish Node [ONLINE]...")

		# Create a VideoCapture object
		# The argument '0' gets the default webcam.
		self.cap = cv2.VideoCapture(0)

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Publish to Image msg
		self.videoFrames_topic = "/video_frames"
		self.videoFrames_pub = rospy.Publisher(self.videoFrames_topic, Image, 
			queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)
		
	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("WebcamPublish Node [OFFLINE]...")

	def cbPublishImage(self):
	
		# Capture frame-by-frame
		# This method returns True/False as well
		# as the video frame.
		self.ret, self.frame = self.cap.read()

		if self.ret == True:
#			# Print debugging information to the terminal
#			rospy.loginfo('publishing video frame')

			# Publish the image.
			# The 'cv2_to_imgmsg' method converts an OpenCV
			# image to a ROS image message
			self.videoFrames_pub.publish(self.bridge.cv2_to_imgmsg(self.frame))

		else:
#			# Print debugging information to the terminal
#			rospy.logwarn('no publishing video frame')
			pass

if __name__ == '__main__':

	# Initialize
	rospy.init_node('WebcamPublish', anonymous=False)
	wp = WebcamPublish()
	
	r = rospy.Rate(60)

	# Camera preview
	while not rospy.is_shutdown():
		wp.cbPublishImage()
		r.sleep()
