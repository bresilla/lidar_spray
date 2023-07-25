#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

fig1, ax1_1 = plt.subplots()
ax1_2 = ax1_1.twinx()
ax1_3 = ax1_1.twinx()
ax1_4 = ax1_1.twinx()

fig2, ax2_1 = plt.subplots()
ax2_2 = ax2_1.twinx()
ax2_3 = ax2_1.twinx()
ax2_4 = ax2_1.twinx()

def callback_2(data1, data2, data3, data4):
    xdata = np.arange(len(data1.ranges))
    line1, = ax1_1.plot(xdata, data1.ranges)
    line2, = ax1_2.plot(xdata, data2.ranges)
    line3, = ax1_3.plot(xdata, data3.ranges)
    line4, = ax1_4.plot(xdata, data4.ranges)
    line1.set_data(xdata, data1.ranges)
    line2.set_data(xdata, data2.ranges)
    line3.set_data(xdata, data3.ranges)
    line4.set_data(xdata, data4.ranges)
    fig1.canvas.draw()
    plt.pause(0.001)

def callback_3(data1, data2, data3, data4):
    xdata = np.arange(len(data1.ranges))
    line1, = ax2_1.plot(xdata, data1.ranges)
    line2, = ax2_2.plot(xdata, data2.ranges)
    line3, = ax2_3.plot(xdata, data3.ranges)
    line4, = ax2_4.plot(xdata, data4.ranges)
    line1.set_data(xdata, data1.ranges)
    line2.set_data(xdata, data2.ranges)
    line3.set_data(xdata, data3.ranges)
    line4.set_data(xdata, data4.ranges)
    fig2.canvas.draw()
    plt.pause(0.001)


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
    ts_2.registerCallback(callback_2)

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
    ts_3.registerCallback(callback_3)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
