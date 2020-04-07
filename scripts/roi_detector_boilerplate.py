#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pickle
import imutils
import copy
import numpy as np
import itertools
import os

class sr_determine_rois():

	def __init__(self):

		self.bridge = CvBridge()
		# get image data from usb_cam from topic /usb_cam/image_rect
		self.image_sub = rospy.Subscriber("/usb_cam/image_rect", Image, self.image_callback)
		self.img = None
		self.ROI_boxes = list() # stores coordinates of cells
	
	# CV_Bridge acts as the middle layer to convert images streamed
	# on rostopics to a format that is compatible with OpenCV
	# image is converted to type mono8 i.e. single channel
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "mono8")
		except CvBridgeError as e:
			print(e)

	def detect_rois(self, minCntrArea, maxCntrArea, lower_thresh, upper_thresh=255):
		'''Detect coordinates black patches on arena'''
		image_copy = self.img.copy()

		blurred = cv2.GaussianBlur(image_copy, (3, 3), 0)
		_, thresh = cv2.threshold(blurred, lower_thresh, upper_thresh, 0)
		_, contours, __ = cv2.findContours(thresh,
																cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		for cnt in contours:
			rect = cv2.minAreaRect(cnt)
			box = cv2.boxPoints(rect)
			box = np.int32(box)
			area = cv2.contourArea(box)
			if area < maxCntrArea and area > maxCntrArea:
				self.ROI_boxes.append(box)

	def sort_rois(self):
		if len(self.ROI_boxes) != 36:
			return
		self.ROI_boxes = sorted(self.ROI_boxes,
														key=lambda box: box[0][1])
		for i in range(0, 36, 6):
			self.ROI_boxes[i:i+6] = sorted(self.ROI_boxes[i:i+6],
																		 key=lambda box: box[0][0])


	def query_yes_no(self, question, default=None):
		
		valid = {"yes": True, "y": True, "ye": True,"no": False, "n": False}
		if default is None:
			prompt = " [Y/N]:\t"
		elif default == "yes":
			prompt = " [Y/N]:\t"
		elif default == "no":
			prompt = " [Y/N]:\t"
		else:
			raise ValueError("Invalid default answer: '%s'" % default)

		while True:
			sys.stdout.write(question + prompt)
			choice = raw_input().lower()
			if default is not None and choice == '':
				return valid[default]
			elif choice in valid:
				return valid[choice]
			else:
				sys.stdout.write("\nPlease respond with 'yes' or 'no' ""(or 'y' or 'n').\n")


	def save_rois(self):
		filename = 'rect_info.pkl'
		package = 'survey_and_rescue'
		with open(os.path.join(os.environ['ROS_PACKAGE_PATH'].split(':')[0],
							package, 'scripts', filename), 'wb+') as file:
			pickle.dump(self.ROI_boxes, file)


	def draw_cell_names(self):
		'''Sorted coordinates must be stored before calling this function in 'rect_info.pkl' file'''
		image_copy = self.img.copy()
		image_copy = cv2.cvtColor(image_copy, cv2.COLOR_GRAY2BGR)
		filename = 'rect_info.pkl'
		package = 'survey_and_rescue'
		cell_coords = None
		with open(os.path.join(os.environ['ROS_PACKAGE_PATH'].split(':')[0],
							package, 'scripts', filename), 'rb') as file:
			cell_coords = pickle.load(file)
		x = 'A'
		y = 1
		for box in cell_coords:
			cv2.drawContours(image_copy, [box], 0, (0, 255, 0), 2)
			cv2.putText(image_copy, x+str(y), (box[0][0]+10, box[0][1]-10), cv2.FONT_HERSHEY_COMPLEX,
									2, (0, 255, 0))
			x = chr(ord(x)+1)
			if x == 'G':
				x = 'A'
				y += 1
		cv2.imshow("ROI Image", image_copy)
		cv2.waitKey(1000)
		cv2.imwrite("ROI_Image.jpg", image_copy)

def main(args):
	# Process flow
	try:
		rospy.init_node('sr_roi_detector', anonymous=False)
		r =	sr_determine_rois()
		lower_thresh = 117
		minCntrArea = 4500
		maxCntrArea = 6500
		while True:
			if r.img is not None:
				no_rois = r.detect_rois(minCntrArea, maxCntrArea, lower_thresh)
				if(len(no_rois) != 36):
					new_thresh_flag = r.query_yes_no("36 cells were not detected, do you want to detect again?",
																						default="yes")
					if(new_thresh_flag):
						param = int(raw_input("What arg do you want to change?\n1. minCntrArea\n"+
														"2. maxCntrArea\n3. lower_thresh\n: "))
						if param == 1:
							minCntrArea = int(raw_input("Enter minCntrArea: "))
						elif param == 2:
							maxCntrArea = int(raw_input("Enter maxCntrArea: "))
						else:
							lower_thresh = int(raw_input("Enter lower_thresh: "))
					else:
						break
				else:
					r.draw_cell_names()
					satis_flag = r.query_yes_no("Are you satisfied with the currently detected ROIs?")
					if satis_flag:
						r.sort_rois()
						r.save_rois()
						cv2.destroyAllWindows()
						break
					else:
						param = int(raw_input("What arg do you want to change?\n1. minCntrArea\n"+
														"2. maxCntrArea\n3. lower_thresh\n: "))
						if param == 1:
							minCntrArea = int(raw_input("Enter minCntrArea: "))
						elif param == 2:
							maxCntrArea = int(raw_input("Enter maxCntrArea: "))
						else:
							lower_thresh = int(raw_input("Enter lower_thresh: "))
			else:
				continue
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)