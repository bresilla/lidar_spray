import rospy
from std_msgs.msg import Int32MultiArray

class SensorProcessor:
    def __init__(self):
        rospy.init_node("sensor_processor")
        rospy.Subscriber("/spray/join", Int32MultiArray, self.callback, queue_size=100)
        self.pub = rospy.Publisher("/spray/sync", Int32MultiArray, queue_size=100)

    def callback(self, msg):
        print(msg)
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    processor = SensorProcessor()
    processor.run()
