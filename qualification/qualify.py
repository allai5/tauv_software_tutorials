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

detected = False
class qualifier(object):
	def __init__(self):
		self.pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)	
		self.sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image", Image, self.callback)
		self.gate_detected = False
	def down(self):
		msg = Twist()
		msg.linear.z = -0.5
		self.pub.publish(msg)
	def stay(self):
		msg = Twist()
		msg.linear.z = 0.0
		self.pub.publish(msg)
	def rotate(self):
		msg = Twist()
		msg.angular.z = 0.6
		self.pub.publish(msg)
	def callback(self, data):
		try:
			bridge = CvBridge()
			cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
			new_image = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
			self.gate_detected = self.template_matching(new_image,template1)
			cv.imshow("New Image", new_image)
			cv.waitKey(50)
		except CvBridgeError as e:
			print(e)

	def template_matching(self, src, template):
		flag = False
		img = src.copy()
		w, h = template.shape[::-1]
		method_str = 'cv.TM_CCOEFF_NORMED'
		method = eval(method_str)
		for i in range(4):
			resize_img = cv.resize(img, None, fx=1/(2**(0.5*i)), fy=1/(2**(0.5*i)), interpolation = cv.INTER_AREA)
			res = cv.matchTemplate(resize_img, template, method)
			if i == 0:
				result = res
			loc = np.where(res >= 0.85) #threshold between 0 and 1
			#print(loc)
			for pt in zip(*loc[::-1]):
				x = (pt[0]*int(2**(0.5*i)), pt[1]*int(2**(0.5*i)))
				y = (pt[0]+w, pt[1]+h)
				cv.rectangle(src, x,y, (255,0,0), 1)
				#print("rect")
				flag = True
				print("rect")
		cv.imshow("Matching Result", result)
		cv.imshow("Detected Image", src)
		#print(flag)
		return flag

	def find_gate(self):
		for i in range(5):
			self.down()
			time.sleep(2)
			print("down")
			self.stay()
			if (self.gate_detected):
				#print("True")
				self.stay()
				time.sleep(1)
				self.center()
				return(True)
			else:
				#print("false")
				initial = time.time()
				t = initial
				while(t < initial + 10):
					#print('rotating')
					self.rotate()
					time.sleep(1)
					t+=1
					#print(t)
					self.stay()
					if (self.gate_detected):
						self.stay()
						self.center()
						return(True)
		return(False)

	def center(self):
		print("CENTER")
	
	def execute(self):
		#navigate down until something is
		print("start")
		found = self.find_gate()
		if(found): 
			print("HALLELUJAH")


def main(args):
	rospy.init_node('qualify', anonymous=True)
	q = qualifier()
	q.execute()
	#rospy.spin()
	#print("loop")

if __name__ == '__main__':
	main(sys.argv)
