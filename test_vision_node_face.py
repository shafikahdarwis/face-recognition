#!/usr/bin/env python

import rospy
import sys
import cv2
import imutils
import argparse

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class TestVisionNode:
	
	def __init__(self):
		rospy.init_node("test_vision_node")
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber("/cv_camera/image_raw/compressed",CompressedImage, self.callback)

		self.faceCascade = cv2.CascadeClassifier("/home/rospika/catkin_ws/src/turtle_vision/haarcascade_frontalface_default.xml")
		# self.eyeCascade = cv2.CascadeClassifier("/home/rospika/catkin_ws/src/turtle_vision/haarcascade_eye.xml")
	
	def callback(self, data):
		# convert ros --> opencv
		self.convert_ros_to_opencv_img(data)
		
		# detect face
		self.track()

		#rospy.loginfo(self.rects)
		frameClone = self.cv_image.copy()

		# loop over the face bounding boxes and draw them
		for rect in self.rects:
			# cv2.rectangle(frameClone, (fX, fY), (fX + fW, fY + fH), (0, 255, 0), 2)
			cv2.rectangle(frameClone, (rect[0], rect[1]), (rect[2], rect[3]), (0, 255, 0), 2)		
	
		cv2.imshow("Image", frameClone)
		cv2.waitKey(1)
	
	def convert_ros_to_opencv_img(self, ros_image):
		self.cv_image = self.bridge.compressed_imgmsg_to_cv2(ros_image)

	def track(self):
		# detect faces in the image and initialize the list of
		# rectangles containing the faces and eyes
		faceRects = self.faceCascade.detectMultiScale(self.cv_image,
			scaleFactor = 1.1, minNeighbors = 5, minSize = (30, 30),
			flags = cv2.CASCADE_SCALE_IMAGE)
		self.rects = []

		# loop over the face bounding boxes
		for (fX, fY, fW, fH) in faceRects:
			# extract the face ROI and update the list of
			# bounding boxes
			faceROI = self.cv_image[fY:fY + fH, fX:fX + fW]
			self.rects.append((fX, fY, fX + fW, fY + fH))
			
			# detect eyes in the face ROI
			# eyeRects = self.eyeCascade.detectMultiScale(faceROI,
			# 	scaleFactor = 1.1, minNeighbors = 10, minSize = (20, 20),
			# 	flags = cv2.CASCADE_SCALE_IMAGE)

			# loop over the eye bounding boxes
			# for (eX, eY, eW, eH) in eyeRects:
			# 	# update the list of boounding boxes
			# 	self.rects.append(
			# 		(fX + eX, fY + eY, fX + eX + eW, fY + eY + eH))

		# return the rectangles representing bounding
		# boxes around the faces and eyes
		# return rects

def main(args):
	vn = TestVisionNode()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Node Shutdown")
	
	cv2.destroyAllWindows()
if __name__ == "__main__":
	main(sys.argv)
