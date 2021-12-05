#!/usr/bin/env python3

################################################################################
## {Description}: Subscribing Image, FPS, Frame Count msg
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
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class WebcamSubscribe:

	def __init__(self):
		
		self.bridge = CvBridge()
		self.fpsValue = Float32()
		self.frameValue = Int64()
		self.image_received = False

		rospy.logwarn("WebcamSubscribe Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to Image msg
		self.videoFrames_topic = "/cv_camera_robot1/image_raw"
		self.videoFrames_sub = rospy.Subscriber(self.videoFrames_topic, Image, self.cbImage)

		# Subscribe to Float32 msg
		self.fpsValue_topic = "/fps_data"
		self.fpsValue_sub = rospy.Subscriber(self.fpsValue_topic, Float32, 
			self.cbFpsValue)

		# Subscribe to Int64 msg
		self.frameValue_topic = "/frame_data"
		self.frameValue_sub = rospy.Subscriber(self.frameValue_topic, Int64, 
			self.cbFrameValue)

		# Allow up to one second to connection
		rospy.sleep(1)

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("CameraPreview Node [OFFLINE]...")
		cv2.destroyAllWindows()

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(msg)

			# comment if the image is mirrored
#			self.cv_image = cv2.flip(self.cv_image, 1)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False

	# Show the output frame
	def cbShowImage(self):
		self.cv_image_clone = imutils.resize(
						self.cv_image.copy(),
						width=320
						)

#		cv2.imshow("CameraPreview", self.cv_image_clone)
		cv2.imshow("CameraPreview", self.cv_image)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
#			self.cbInfo()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_DUPLEX
		fontScale = 1
		color = (255, 255, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

#		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
#			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
#			color, thickness, lineType, bottomLeftOrigin)

		cv2.putText(self.cv_image, "%d:%.2f" % (self.frameValue, self.fpsValue), (10, 20), 
			fontFace, fontScale, color, thickness, lineType, 
			bottomLeftOrigin)

	def cbFpsValue(self, msg):
		self.fpsValue = msg.data

	def cbFrameValue(self, msg):
		self.frameValue = msg.data

if __name__ == '__main__':

	# Initialize
	rospy.init_node('WebcamSubscribe', anonymous=False)
	ws = WebcamSubscribe()
	
	r = rospy.Rate(10)

	# Camera preview
	while not rospy.is_shutdown():
		ws.cbPreview()
		r.sleep()
