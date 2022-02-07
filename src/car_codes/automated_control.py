#!/usr/bin/env python3

import json
from pynput import keyboard
import time
from example.src.RcBrainThread import RcBrainThread
from std_msgs.msg import String

import rospy


class Control():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread.
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.

        """

        self.rcBrain = RcBrainThread()

        rospy.init_node('EXAMPLEnode', anonymous=False)
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        keyMsg = 'p.r'
        self._send_command(keyMsg)
        keyMsg = 'p.p'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)
        keyMsg = 'p.a'
        self._send_command(keyMsg)
        keyMsg = 'p.w'
        self._send_command(keyMsg)

        # self.publisher.publish('{"action":"2","steerAngle":0.0}')
        # keyMsg = 'p.p'
        # self._send_command(keyMsg)
        #
        # self.publisher.publish('{"action":"1","steerAngle":13.0}')




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


if __name__ == '__main__':
    try:
        nod = Control()
    except rospy.ROSInterruptException:
        pass