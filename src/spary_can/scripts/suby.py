# DO NOT skip the next commented line
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import can

can.rc['interface'] = 'socketcan'
can.rc['channel'] = 'can0'
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


def callback_235(data):
    print(data.ranges)
    id = 0x0C218099
    by = [0, 25, 0, 1, 3, 1, 4, 1]
    send2can(id, by)


def callback_236(data):
    print(data.ranges)
    id = 0x0C228099
    by = [0, 25, 0, 34, 3, 67, 4, 1]
    send2can(id, by)


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/two/cloud_1", LaserScan, callback_235)
    rospy.Subscriber("/three/cloud_1", LaserScan, callback_236)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
