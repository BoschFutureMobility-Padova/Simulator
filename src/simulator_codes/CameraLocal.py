
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
    """
    :param data: sensor_msg array containing the image in the Gazsbo format
    :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
    """
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("Frame preview", cv_image)
    key = cv2.waitKey(1)


bridge = CvBridge()
cv_image = np.zeros((640, 480))
rospy.init_node('CAMnod', anonymous=True)
image_sub = rospy.Subscriber("/automobile/rcCar/camera_follow/image_raw", Image, callback)
rospy.spin()


