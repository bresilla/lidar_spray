import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
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
        print(f"Message content: {msg}")
    except can.CanError:
        print("Message NOT sent")

def publisher():
    pub = rospy.Publisher('/heartbeat', Int16, queue_size=10)
    rospy.init_node('heartbeat', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        id = 0x14FF8099
        by = 0x01
        send2can(id, by)
        rospy.loginfo(by)
        pub.publish(by)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
