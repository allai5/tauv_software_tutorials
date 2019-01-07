import rospy
import cv2 as cv
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time

def image_publisher():
    pub = rospy.Publisher('camera_image', Image, queue_size=100)
    rospy.init_node('image_publisher', anonymous = True)
    bridge = CvBridge()

    cap = cv.VideoCapture('test480.mp4')
    ret, frame = cap.read()
    cv.imshow('Frame', frame)

    while True:
        cv.imshow('Frame', frame)
        ret, frame = cap.read()

        if frame is not None:
            frame = np.uint8(frame)
        image_msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        pub.publish(image_msg)

        
        cv.waitKey(50)   # was 20 but playing video fiels too fast
        # print "key pressed: " + str(key)
        # exit on ESC, you may want to uncomment the print to know which key is ESC for you
    cap.release()
    cv.destroyAllWindows()



if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass