#!/usr/bin/env python
# !coding=utf-8
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import json
from example.src.RcBrainThread import RcBrainThread
from std_msgs.msg import String


class Control():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread.
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.

        """
        rospy.init_node('LaneDetectionNode', anonymous=False)

        self.bridge = CvBridge()

        self.rcBrain = RcBrainThread()

        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)


        self.publisher.publish('{"action":"2","steerAngle":0.0}')
        keyMsg = 'p.p'
        self._send_command(keyMsg)

        self.publisher.publish('{"action":"1","steerAngle":10.0}')


    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver.

        Parameters
        ----------
        inP : Pipe
            Input pipe.
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:
            command = json.dumps(command)
            self.publisher.publish(command)

    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(1)

    def image_callback(self, img_msg):
        rospy.loginfo(img_msg.header)
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        self.lane_detection(cv_image)

    def lane_detection(self, frame):
        stencil = np.zeros_like(frame[:, :, 0])

        # specify coordinates of the polygon
        polygon = np.array([[0, 480], [120, 300], [530, 300], [640, 480]])
        cv2.fillConvexPoly(stencil, polygon, 1)
        img = cv2.bitwise_and(frame[:, :, 0], frame[:, :, 0], mask=stencil)

        # apply image thresholding
        _, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, 30, maxLineGap=200)

        dmy = frame.copy()

        angls = list()
        # draw Hough lines
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)

                v1_theta = math.atan2(y1, x1)
                v2_theta = math.atan2(y2, x2)
                r = (v2_theta - v1_theta) * (180.0 / math.pi)
                angls.append(r)

        self.show_image(dmy)
        neg = list(filter(lambda x: (x < 0), angls))
        pos = list(filter(lambda x: (x > 0), angls))
        if len(neg) > 10 and len(pos) < 2:
            self.publisher.publish('{"action":"2","steerAngle":-20.0}')
            print("left")
        elif len(neg) < 2 and len(pos) > 10:
            self.publisher.publish('{"action":"2","steerAngle":20.0}')
            print("right")

        else:
            print("straight")
            self.publisher.publish('{"action":"2","steerAngle":0.0}')


if __name__ == '__main__':
    c = Control()
    sub_cam_info = rospy.Subscriber("automobile/image_raw", Image, c.image_callback)
    cv2.namedWindow("Image Window", 1)

    while not rospy.is_shutdown():
        rospy.spin()
    # lane_detection(frames[4:25])