#!/usr/bin/env python
# !coding=utf-8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

bridge = CvBridge()
out = None


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    i = 0

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    show_image(cv_image)
    filename = input('Enter your name:')
    cv2.imwrite(filename, gray)
    


rospy.init_node('photo', anonymous=True)
sub_cam_info = rospy.Subscriber("automobile/image_raw", Image, image_callback)
# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()




