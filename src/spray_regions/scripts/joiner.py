#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import Int32MultiArray
import threading

class RegionsNode:
    def __init__(self):
        rospy.init_node('regions', anonymous=True)
        self.pub_all = rospy.Publisher('/spray/join', Int32MultiArray, queue_size=2)
        self.two = message_filters.Subscriber('/spray/mean/two', Int32MultiArray)
        self.three = message_filters.Subscriber('/spray/mean/three', Int32MultiArray)
        self.rate = rospy.Rate(10)
        self.two_buffer = []
        self.three_buffer = []

        self.ts_2 = message_filters.ApproximateTimeSynchronizer(
            [self.two, self.three],
            queue_size=1,
            slop=1,
            allow_headerless=True,
            reset=False,
        )
        self.ts_2.registerCallback(self.callback_one)

    def callback_one(self, two, three):
        self.rate.sleep()
        x = list(two.data + three.data)
        print(f"{x}")
        self.pub_all.publish(data=x)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RegionsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
