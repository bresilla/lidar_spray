import rospy
from std_msgs.msg import Int32MultiArray, Float32, String
import threading
import time

class SensorProcessor:
    def __init__(self):
        self.sensor_readings = []
        self.start_collecting = False
        self.wait_time = None
        rospy.init_node("sensor_processor")
        rospy.Subscriber("/spray/join", Int32MultiArray, self.callback, queue_size=100)
        self.pub = rospy.Publisher("/spray/sync", Int32MultiArray, queue_size=100)
        rospy.Subscriber("/spray/speed_set", Float32, self.speed_callback, queue_size=1)
        rospy.Subscriber('/spray/off', String, self.shutdown_callback, queue_size=1)

    def calc_position(self, distance_between_sensors, speed_of_vehicle_km_per_s):
        # Convert the speed of the vehicle from km/s to m/s
        speed_of_vehicle_m_per_s = speed_of_vehicle_km_per_s * (1000 / 3600)
        # Calculate the relative velocity
        relative_velocity = speed_of_vehicle_m_per_s - 0  # Assuming sensors are moving together in the vehicle
        # Calculate the time
        time_to_same_position = distance_between_sensors / relative_velocity
        return time_to_same_position
    
    def calc_buffer(self, time_to_same_position, time_interval):
        return int(time_to_same_position * (time_interval * 100))

    def append_callback(self, msg):
        self.sensor_readings.append(msg.data)
        self.start_collecting = True

    def publish_callback(self):
        if self.wait_time > 0: 
            self.wait_time -= 1
            return
        processed_readings = self.sensor_readings.pop(0)
        print(f"Processed: {processed_readings}")
        self.pub.publish(Int32MultiArray(data=processed_readings))

    def callback(self, msg):
        if self.wait_time is None: return
        self.append_callback(msg)
        self.publish_callback()

    def speed_callback(self, msg):
        position = self.calc_position(1.5, msg.data)
        buffer = self.calc_buffer(position, 0.1)
        print(f"calculate position: {position}")
        print(f"calculate buffer: {buffer}")
        self.wait_time = buffer

    def shutdown_callback(self, msg):
        print(msg.data)
        self.wait_time = None
        self.sensor_readings = []
        self.start_collecting = False


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    processor = SensorProcessor()
    processor.run()
