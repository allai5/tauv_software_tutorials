import sys
import rospy
import roslib
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist
import time

template1 = cv.imread('real_bar3.png')
template1 = cv.cvtColor(template1,cv.COLOR_BGR2GRAY)

def template_matching(src, template):
	#frame = cv.cvtColor(src,cv.COLOR_BGR2GRAY)
	img = src.copy()
	w, h = template.shape[::-1]
	method_str = 'cv.TM_CCOEFF_NORMED'
	method = eval(method_str)
	for i in range(4):
		resize_img = cv.resize(img, None, fx=1/(2**(0.5*i)), fy=1/(2**(0.5*i)), interpolation = cv.INTER_AREA)
		#print(resize_img.shape)
		res = cv.matchTemplate(resize_img, template, method)
		if i == 0:
			result = res
		loc = np.where(res >= 0.85)
		for pt in zip(*loc[::-1]):
			x = (pt[0]*int(2**(0.5*i)), pt[1]*int(2**(0.5*i)))
			y = (pt[0]+w, pt[1]+h)
			cv.rectangle(src, x,y, (255,0,0), 1)
			print("rectangle")
	

	cv.imshow("Matching Result", result)
	cv.imshow("Detected Image", src)
	
	#print(loc)
	#determine middle of the image
	#middle x value, middle y value
	#match coordinates
def callback(data):
	try:
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		new_image = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
		template_matching(new_image,template1)
		cv.waitKey(50)
	except CvBridgeError as e:
		print(e)
def image_listener():
	rospy.Subscriber("/rexrov/rexrov/camera/camera_image", Image, callback)
	rospy.spin()

def main(args):
	#rospy.init_node('match', anonymous=True)
	orig = cv.imread('rip.png')
	img = cv.cvtColor(orig,cv.COLOR_BGR2GRAY)
	
	while(True):
		template_matching(img,template1)
		cv.waitKey(50)
	#	image_listener()
if __name__ == '__main__':
	main(sys.argv)
