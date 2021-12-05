#!/usr/bin/env python3

################################################################################
## {Description}: Publishing Image, FPS, Frame Count msg (Faster; Multithreading)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
import sys
import cv2
import time
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy


class WebcamPublishFaster:

	def __init__(self):
	
		self.bridge = CvBridge()
		self.fpsValue = Float32()
		self.frameValue = Int64()

		# used to record the frame no
		self.frameValue.data = 0

		# used to record the time when we processed last frame
		self.prev_frame_time = 0

		# used to record the time at which we processed current frame
		self.new_frame_time = 0

		rospy.logwarn("WebcamPublishFaster Node [ONLINE]...")

		# Create a VideoCapture object
		# The argument '0' gets the default webcam.
		self.cap = WebcamVideoStream(src=0).start()

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Publish to Image msg
		self.videoFrames_topic = "/video_frames"
		self.videoFrames_pub = rospy.Publisher(self.videoFrames_topic, Image, 
			queue_size=10)

		# Publish to Float32 msg
		self.fpsValue_topic = "/fps_data"
		self.fpsValue_pub = rospy.Publisher(self.fpsValue_topic, Float32, 
			queue_size=10)

		# Publish to Int64 msg
		self.frameValue_topic = "/frame_data"
		self.frameValue_pub = rospy.Publisher(self.frameValue_topic, Int64, 
			queue_size=10)

		# Allow up to one second to connection
		rospy.sleep(1)
		
	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("WebcamPublishFaster Node [OFFLINE]...")

	def cbPublishImage(self):
	
		# Capture frame-by-frame
		# This method returns True/False as well
		# as the video frame.
		self.frame = self.cap.read()

		self.frame = imutils.resize(self.frame, width=320)

		self.frameValue.data += 1

		if self.frame is not None:
#			# Print debugging information to the terminal
#			rospy.loginfo('publishing video frame')

			# time when we finish processing for this frame
			self.new_frame_time = time.time()

			# Calculating the fps

			# fps will be number of frame processed in given time frame
			# since their will be most of time error of 0.001 second
			# we will be subtracting it to get more accurate result
			self.fpsValue.data = 1/(self.new_frame_time - self.prev_frame_time)
			self.prev_frame_time = self.new_frame_time

#			# converting the fps into integer
#			self.fps = int(self.fps)

#			# converting the fps to string so that we can display it on frame
#			# by using putText function
#			self.fps = str(self.fps)

			# Publish the image.
			# The 'cv2_to_imgmsg' method converts an OpenCV
			# image to a ROS image message
			self.videoFrames_pub.publish(self.bridge.cv2_to_imgmsg(self.frame))

##			# Print debugging information to the terminal
#			rospy.loginfo(self.fps)

			self.fpsValue_pub.publish(self.fpsValue.data)
			self.frameValue_pub.publish(self.frameValue.data)

		else:
#			# Print debugging information to the terminal
#			rospy.logwarn('no publishing video frame')
			pass

if __name__ == '__main__':

	# Initialize
	rospy.init_node('WebcamPublishFaster', anonymous=False)
	wpf = WebcamPublishFaster()
	
	r = rospy.Rate(10)

	# Camera preview
	while not rospy.is_shutdown():
		wpf.cbPublishImage()
		r.sleep()
