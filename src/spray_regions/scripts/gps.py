#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray, Int16
import can

can.rc['interface'] = 'socketcan'
# can.rc['channel'] = 'can0'
can.rc['channel'] = 'vcan1'
can.rc['bitrate'] = 500000

from can.interface import Bus
bus = Bus()

def _map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    np.interp(x, (x.min(), x.max()), (-1, +1))

def send2can(id, by):
    msg = can.Message(arbitration_id=id, data=by, is_extended_id=True)
    try:
        bus.send(msg)
        # print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

pub_2 = rospy.Publisher('/two/feedback', Int16, queue_size=1)
pub_3 = rospy.Publisher('/three/feedback', Int16, queue_size=1)

def callback_two(data):
    r = rospy.Rate(10) 
    r.sleep()
    id_two = 0x0C218099
    x = np.asarray(data.data)
    newdata = np.interp(x, (x.min(), x.max()), (0, 254)).astype(int)
    print(f"two: {newdata}")
    send2can(id_two, list(newdata))
    global pub_2
    pub_2.publish(len(newdata))

def callback_three(data):
    r = rospy.Rate(10) 
    r.sleep()
    id_three = 0x0C228099
    x = np.asarray(data.data)
    newdata = np.interp(x, (x.min(), x.max()), (0, 254)).astype(int)
    print(f"three: {newdata}")
    send2can(id_three, list(newdata))
    global pub_3
    pub_3.publish(len(newdata))

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
