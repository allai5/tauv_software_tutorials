import rospy
import cv2 as cv
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

bridge = CvBridge()
def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        new_image = cv.cvtColor(cv_image,cv.COLOR_BGR2GRAY)
        cv.imshow("New Image", new_image)
        cv.waitKey(50)
    except CvBridgeError as e:
        print(e)

def image_listener():
	rospy.init_node("listener", anonymous = True)
	rospy.Subscriber("camera_image", Image, callback)
	rospy.spin()

if __name__ == '__main__':
	image_listener()