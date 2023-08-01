#!/usr/bin/env python

import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, Float32, String
import argparse

class SpeedSet:
    def __init__(self):
        self.flip_2 = False
        self.flip_3 = True
        self.upper = 20
        self.lower = 20
        self.num = 9
        self.pub_2 = rospy.Publisher('/spray/mean/two', Int32MultiArray, queue_size=10)
        self.pub_3 = rospy.Publisher('/spray/mean/three', Int32MultiArray, queue_size=10)
        self.speed_pub = rospy.Publisher('/spray/speed_set', Float32, queue_size=10)
        self.off = rospy.Publisher('/spray/off', String, queue_size=10)


    def clip_array(self, array, upper=1, lower=1):
        total_percentage = upper + lower
        if total_percentage > 100:
            upper = (upper / total_percentage) * 100
            lower = (lower / total_percentage) * 100
        upper = np.clip(upper, 1, 99)
        lower = np.clip(lower, 1, 99)
        upper = int((len(array) * upper / 100))
        lower = int((len(array) * lower / 100))
        arr = array[upper:-lower]
        throw = len(arr) % self.num
        arr2 = arr[:-throw]
        return arr2

    def clip_thresh(self, array, threshold=1):
        array = np.where(array > threshold, 0, 1)
        return array

    def map_values(self, arr):
        min_val = 0
        max_val = 1050
        new_min_val = 254
        new_max_val = 0

        mapped_array = [int((x - min_val) * (new_max_val - new_min_val) / (max_val - min_val) + new_min_val) for x in arr]
        return mapped_array

    def get_values(self, ranges, upper=1, lower=1, flipp=False):
        array = np.asarray(list(ranges.ranges))
        array = array[::-1] if flipp else array
        array_maped = self.map_values(array)
        array_clipped = self.clip_array(array_maped, upper=upper, lower=lower)
        arrays = np.asarray(np.array_split(array_clipped, self.num))
        means = arrays.mean(axis=1).astype(int)
        return list(means), array_maped, array_clipped

    def callback_two(self, two1, two2, two3, two4):
        m_1, a1, a2 = self.get_values(two1, flipp=self.flip_2, upper=self.upper, lower=self.lower)
        m_2, a1, a2 = self.get_values(two2, flipp=self.flip_2, upper=self.upper, lower=self.lower)
        m_3, a1, a2 = self.get_values(two3, flipp=self.flip_2, upper=self.upper, lower=self.lower)
        m_4, a1, a2 = self.get_values(two4, flipp=self.flip_2, upper=self.upper, lower=self.lower)
        means = np.asarray([m_1, m_2, m_3, m_4])
        means = list(means.mean(axis=0).astype(int))
        print(f"\t\t2\t\t: {means}")
        self.pub_2.publish(data=means)

    def callback_three(self, three1, three2, three3, three4):
        m_1, a1, a2 = self.get_values(three1, flipp=self.flip_3, upper=self.upper, lower=self.lower)
        m_2, a1, a2 = self.get_values(three2, flipp=self.flip_3, upper=self.upper, lower=self.lower)
        m_3, a1, a2 = self.get_values(three3, flipp=self.flip_3, upper=self.upper, lower=self.lower)
        m_4, a1, a2 = self.get_values(three4, flipp=self.flip_3, upper=self.upper, lower=self.lower)
        means = np.asarray([m_1, m_2, m_3, m_4])
        means = list(means.mean(axis=0).astype(int))
        print(f"\t\t3\t\t: {means}")
        self.pub_3.publish(data=means)

    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='ROS Node with Arguments')
        parser.add_argument('--speed', type=int, default=2, help='Argument 2 (str)')
        args = parser.parse_args()
        return args
    
    def shutdown_callback(self):
        rospy.loginfo("Shutting down... Sending final message.")
        self.off.publish("Node is shutting down.")

    def run(self, speed=1.2):
        rospy.init_node('regions', anonymous=True)
        args = self.parse_arguments()
        self.speed_pub.publish(args.speed)

        two1 = message_filters.Subscriber("/two/cloud_1", LaserScan)
        two2 = message_filters.Subscriber("/two/cloud_2", LaserScan)
        two3 = message_filters.Subscriber("/two/cloud_3", LaserScan)
        two4 = message_filters.Subscriber("/two/cloud_4", LaserScan)
        ts_2 = message_filters.ApproximateTimeSynchronizer(
            [two1, two2, two3, two4],
            queue_size=1,
            slop=1,
            allow_headerless=True,
            reset=False,
        )
        ts_2.registerCallback(self.callback_two)

        three1 = message_filters.Subscriber("/three/cloud_1", LaserScan)
        three2 = message_filters.Subscriber("/three/cloud_2", LaserScan)
        three3 = message_filters.Subscriber("/three/cloud_3", LaserScan)
        three4 = message_filters.Subscriber("/three/cloud_4", LaserScan)
        ts_3 = message_filters.ApproximateTimeSynchronizer(
            [three1, three2, three3, three4],
            queue_size=1,
            slop=1,
            allow_headerless=True,
            reset=False,
        )
        ts_3.registerCallback(self.callback_three)
        rospy.on_shutdown(self.shutdown_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        speed_set = SpeedSet()
        speed_set.run()
    except rospy.ROSInterruptException:
        pass
