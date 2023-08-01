#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
import can
import socket

class CANPublisher:
    def __init__(self):
        self.ids = [0x0C218099, 0x0C228099, 0x0C238099]
        self.bus = None  # Will be initialized in start_can_interface()
        self.setup_can()
        rospy.init_node('can', anonymous=True)
        rospy.Subscriber("/spray/sync", Int32MultiArray, self.callback, queue_size=1)
        rospy.spin()

    def setup_can(self):
        channel = "vcan0" if socket.gethostname() == "focal" else "can0"
        try:
            self.bus = can.interface.Bus(channel=channel, bustype="socketcan")
        except can.CanError as e:
            rospy.logerr(f"Error initializing CAN interface: {e}")
            self.bus = None

    def send2can(self, can_id, data):
        if self.bus is not None:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
            try:
                self.bus.send(msg)
            except can.CanError:
                rospy.logerr("Message NOT sent")

    def callback(self, data):
        x = np.asarray(data.data)
        rospy.loginfo(f"all: {x}")
        x = np.pad(x, (0, 24 - len(x)), 'constant')
        x = np.array_split(x, 3)
        for i in range(len(x)):
            self.send2can(self.ids[i], list(x[i]))

if __name__ == '__main__':
    try:
        can_publisher = CANPublisher()
    except rospy.ROSInterruptException:
        pass
