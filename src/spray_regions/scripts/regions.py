#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

flip_2 = False
flip_3 = True
upper = 20
lower = 20
num = 8

pub_2 = rospy.Publisher('/two/regions', Int32MultiArray, queue_size=10)
pub_3 = rospy.Publisher('/three/regions', Int32MultiArray, queue_size=10)

def clip_array(array, upper=1, lower=1):
    total_percentage = upper + lower
    if total_percentage > 100:
        upper = (upper / total_percentage) * 100
        lower = (lower / total_percentage) * 100
    upper = np.clip(upper, 1, 99)
    lower = np.clip(lower, 1, 99)
    upper = int((len(array)*upper/100))
    lower = int((len(array)*lower/100))
    arr = array[upper:-lower]
    throw = len(arr)%num
    arr2 = arr[:-throw]
    return arr2

#create function that maps each value of array to 0 if biger than threshold or 1 if smaller than threshold
def clip_thresh(array, threshold=1):
    array = np.where(array > threshold, 0, 1)
    return array

def map_values(arr):
    min_val = 0
    max_val = 1050
    new_min_val = 254
    new_max_val = 0

    mapped_array = [int((x - min_val) * (new_max_val - new_min_val) / (max_val - min_val) + new_min_val) for x in arr]
    return mapped_array

def get_values(ranges, upper=1, lower=1, flipp=False):
    array = np.asarray(list(ranges.ranges))
    array = array[::-1] if flipp else array
    # array_thresh = clip_thresh(array2, threshold=1000)
    array_maped = map_values(array)
    array_clipped = clip_array(array_maped, upper=upper, lower=lower)
    arrays = np.asarray(np.array_split(array_clipped, num))
    means = arrays.mean(axis=1).astype(int)
    return list(means), array_maped, array_clipped

def callback_two(two1, two2, two3, two4):
    global pub_2
    m_1, a1, a2 = get_values(two1, flipp=flip_2, upper=upper, lower=lower)
    # print(f"original: {two1.ranges}")
    # print(f"mapped: {a1}")
    # print(f"clipped: {a2}")
    m_2, a1, a2 = get_values(two2, flipp=flip_2, upper=upper, lower=lower)
    m_3, a1, a2 = get_values(two3, flipp=flip_2, upper=upper, lower=lower)
    m_4, a1, a2 = get_values(two4, flipp=flip_2, upper=upper, lower=lower)
    means = np.asarray([m_1, m_2, m_3, m_4])
    means = list(means.mean(axis=0).astype(int))
    print(f"\t\t2\t\t: {means}")
    pub_2.publish(data=means)

def callback_three(three1, three2, three3, three4):
    global pub_3
    m_1, a1, a2 = get_values(three1, flipp=flip_3, upper=upper, lower=lower)
    # print(f"original: {three1.ranges}")
    # print(f"mapped: {a1}")
    # print(f"clipped: {a2}")
    m_2, a1, a2 = get_values(three2, flipp=flip_3, upper=upper, lower=lower)
    m_3, a1, a2 = get_values(three3, flipp=flip_3, upper=upper, lower=lower)
    m_4, a1, a2 = get_values(three4, flipp=flip_3, upper=upper, lower=lower)
    means = np.asarray([m_1, m_2, m_3, m_4])
    means = list(means.mean(axis=0).astype(int))
    print(f"\t\t3\t\t: {means}")
    pub_3.publish(data=means)


def main():
    rospy.init_node('regions', anonymous=True)

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
    ts_2.registerCallback(callback_two)

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
    ts_3.registerCallback(callback_three)

    rospy.spin() 
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
