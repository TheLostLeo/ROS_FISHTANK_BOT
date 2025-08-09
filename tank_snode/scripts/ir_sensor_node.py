#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random
import time
"""ROS Node for IR Sensor helping in edge detection"""

"""Simulated IR Sensor Node for Tank Cleaning Robot"""
class SimulatedIRSensor:
    def __init__(self):
        self.consecutive_safe_readings = 0
        self.edge_detection_probability = 0.05
        self.edge_duration_min = 3
        self.edge_duration_max = 8
        self.edge_countdown = 0
        self.is_cleaning = False
        self.last_status = None

    def tank_control_callback(self, msg):
        command = msg.data
        if command == "START_CLEANING":
            self.is_cleaning = True
            print("IR Sensor: Activated")
        elif command == "STOP_CLEANING":
            self.is_cleaning = False
            print("IR Sensor: Deactivated")

    def read_sensor(self):
        if self.edge_countdown > 0:
            self.edge_countdown -= 1
            self.consecutive_safe_readings = 0
            return "EDGE"
        
        if self.consecutive_safe_readings > 20:
            edge_chance = min(0.15, self.edge_detection_probability * (self.consecutive_safe_readings / 20))
        else:
            edge_chance = self.edge_detection_probability
            
        if random.random() < edge_chance:
            self.edge_countdown = random.randint(self.edge_duration_min, self.edge_duration_max)
            self.consecutive_safe_readings = 0
            return "EDGE"
        else:
            self.consecutive_safe_readings += 1
            return "SAFE"

def sensor_node():
    rospy.init_node('sensor_node')
    pub = rospy.Publisher('/edge_status', String, queue_size=10)
    rate = rospy.Rate(10) 
    ir_sensor = SimulatedIRSensor()
    
    rospy.Subscriber('/tank_control', String, ir_sensor.tank_control_callback)
    
    print("IR Sensor Node is ready and waiting for commands")

    reading_count = 0
    
    while not rospy.is_shutdown():
        status = ir_sensor.read_sensor()
        pub.publish(status)
        
        reading_count += 1
        if ir_sensor.is_cleaning and status != ir_sensor.last_status:
            if status == "EDGE":
                print("IR Sensor: EDGE DETECTED! Tank should avoid obstacle")
            elif status == "SAFE" and ir_sensor.last_status == "EDGE":
                print("IR Sensor: Path clear - obstacle avoided")
            
        ir_sensor.last_status = status
        rate.sleep()

if __name__ == '__main__':
    try:
        sensor_node()
    except rospy.ROSInterruptException:
        pass
