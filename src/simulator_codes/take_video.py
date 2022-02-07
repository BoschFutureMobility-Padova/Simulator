#!/usr/bin/env python
# !coding=utf-8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
out = None


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image)
    global out
    if not out:
        height, width, channels = cv_image.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter("/home/sepideh/workspace/bosch/Brain/Simulator/filename.mp4", fourcc, 3, (width, height))
    else:
        out.write(cv_image)


rospy.init_node('video', anonymous=True)
sub_cam_info = rospy.Subscriber("/automobile/rcCar/camera_follow/image_raw", Image, image_callback)
# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
out.release()




