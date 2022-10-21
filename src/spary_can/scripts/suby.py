#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import LaserScan
import can

can.rc['interface'] = 'socketcan'
# can.rc['channel'] = 'can0'
can.rc['channel'] = 'vcan1'
can.rc['bitrate'] = 500000

from can.interface import Bus
bus = Bus()


def send2can(id, by):
    msg = can.Message(arbitration_id=id, data=by, is_extended_id=True)
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")


def callback(two_cloud_1, three_cloud_1):
    print(len(two_cloud_1.ranges))
    id_two = 0x0C218099
    id_three = 0x0C228099
    bytes_two = [0, 25, 0, 1, 3, 1, 4, 1]
    send2can(id_two, bytes_two)
    send2can(id_three, bytes_two)


def main():
    rospy.init_node('listener', anonymous=True)
    two_cloud_1 = message_filters.Subscriber("/two/cloud_1", LaserScan)
    three_cloud_1 = message_filters.Subscriber("/three/cloud_1", LaserScan)

    ts = message_filters.ApproximateTimeSynchronizer([two_cloud_1, three_cloud_1], 1, 1)
    ts.registerCallback(callback)
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
