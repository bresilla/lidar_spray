#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

flip_2 = False
flip_3 = True

pub_2 = rospy.Publisher('/two/regions', Int32MultiArray, queue_size=10)
pub_3 = rospy.Publisher('/three/regions', Int32MultiArray, queue_size=10)

def get_values(ranges, upper=1, lower=1, flipp=False):
    array = np.asarray(list(ranges.ranges))
    array = array[::-1] if flipp else array
    upper = np.clip(upper, 1, 99)
    lower = np.clip(lower, 1, 98)
    upper = int((len(array)*upper/100)/2)
    lower = int((len(array)*lower/100)/2)
    array = array[lower:-upper]
    array = np.asarray(np.array_split(array, 8))
    means = array.mean(axis=1)
    return array, list(means)

def callback_two(two1, two2, two3, two4):
    global pub_2
    a_1, m_1 = get_values(two1, flipp=flip_2)
    a_2, m_2 = get_values(two2, flipp=flip_2)
    a_3, m_3 = get_values(two3, flipp=flip_2)
    a_4, m_4 = get_values(two4, flipp=flip_2)
    means = np.asarray([m_1, m_2, m_3, m_4])
    means = list(means.mean(axis=0).astype(int))
    pub_2.publish(data=means)

def callback_three(three1, three2, three3, three4):
    global pub_3
    a_1, m_1 = get_values(three1, flipp=flip_3)
    a_2, m_2 = get_values(three2, flipp=flip_3)
    a_3, m_3 = get_values(three3, flipp=flip_3)
    a_4, m_4 = get_values(three4, flipp=flip_3)
    means = np.asarray([m_1, m_2, m_3, m_4])
    means = list(means.mean(axis=0).astype(int))
    pub_3.publish(data=means)


def main():
    rospy.init_node('regions', anonymous=True)

    two1 = message_filters.Subscriber("/two/cloud_1", LaserScan)
    two2 = message_filters.Subscriber("/two/cloud_2", LaserScan)
    two3 = message_filters.Subscriber("/two/cloud_3", LaserScan)
    two4 = message_filters.Subscriber("/two/cloud_4", LaserScan)
    ts_2 = message_filters.ApproximateTimeSynchronizer([two1, two2, two3, two4], 1, 1, 1, 1)
    ts_2.registerCallback(callback_two)

    three1 = message_filters.Subscriber("/three/cloud_1", LaserScan)
    three2 = message_filters.Subscriber("/three/cloud_2", LaserScan)
    three3 = message_filters.Subscriber("/three/cloud_3", LaserScan)
    three4 = message_filters.Subscriber("/three/cloud_4", LaserScan)
    ts_3 = message_filters.ApproximateTimeSynchronizer([three1, three2, three3, three4], 1, 1, 1, 1)
    ts_3.registerCallback(callback_three)

    rospy.spin() 
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
