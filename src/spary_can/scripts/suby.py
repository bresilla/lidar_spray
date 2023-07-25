#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
import can

can.rc['interface'] = 'socketcan'
# can.rc['channel'] = 'can0'
can.rc['channel'] = 'vcan0'
can.rc['bitrate'] = 500000

from can.interface import Bus
bus = Bus()

def send2can(id, by):
    msg = can.Message(arbitration_id=id, data=by, is_extended_id=True)
    try:
        bus.send(msg)
    except can.CanError:
        print("Message NOT sent")

def callback_two(data):
    r = rospy.Rate(10) 
    r.sleep()
    id_two = 0x0C218099
    x = np.asarray(data.data)
    print(f"two: {x}")
    send2can(id_two, list(x))

def callback_three(data):
    r = rospy.Rate(10) 
    r.sleep()
    id_three = 0x0C228099
    x = np.asarray(data.data)
    print(f"three: {x}")
    send2can(id_three, list(x))

def main():
    rospy.init_node('can', anonymous=True)
    rospy.Subscriber("/two/regions", Int32MultiArray, callback_two, queue_size=1)
    rospy.Subscriber("/three/regions", Int32MultiArray, callback_three, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
