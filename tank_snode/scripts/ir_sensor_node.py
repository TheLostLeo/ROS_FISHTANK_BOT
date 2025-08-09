#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import random
import time

# DEMO MODE: Simulating IR sensor without GPIO
# This version generates random edge detection for demonstration

class SimulatedIRSensor:
    def __init__(self):
        self.consecutive_safe_readings = 0
        self.edge_detection_probability = 0.05  # 5% chance per reading
        self.edge_duration_min = 3  # minimum edge readings in a row
        self.edge_duration_max = 8  # maximum edge readings in a row
        self.edge_countdown = 0
        self.is_cleaning = False  # Track if cleaning is active
        self.last_status = None  # Track last status to detect changes
        
    def tank_control_callback(self, msg):
        """Track cleaning status to control output"""
        command = msg.data
        if command == "START_CLEANING":
            self.is_cleaning = True
            print("IR Sensor: Activated - monitoring for edges")
        elif command == "STOP_CLEANING":
            self.is_cleaning = False
            print("IR Sensor: Deactivated - standing by")
        
    def read_sensor(self):
        """Simulate IR sensor reading with realistic edge detection patterns"""
        
        # If we're in an edge detection sequence, continue it
        if self.edge_countdown > 0:
            self.edge_countdown -= 1
            self.consecutive_safe_readings = 0
            return "EDGE"
        
        # After being safe for a while, increase chance of edge detection
        if self.consecutive_safe_readings > 20:  # After 2 seconds of safe readings
            edge_chance = min(0.15, self.edge_detection_probability * (self.consecutive_safe_readings / 20))
        else:
            edge_chance = self.edge_detection_probability
            
        # Simulate edge detection
        if random.random() < edge_chance:
            # Start an edge detection sequence
            self.edge_countdown = random.randint(self.edge_duration_min, self.edge_duration_max)
            self.consecutive_safe_readings = 0
            return "EDGE"
        else:
            self.consecutive_safe_readings += 1
            return "SAFE"

def sensor_node():
    rospy.init_node('sensor_node')
    pub = rospy.Publisher('/edge_status', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # DEMO MODE: Create simulated sensor
    ir_sensor = SimulatedIRSensor()
    
    # Subscribe to tank control to track cleaning status
    rospy.Subscriber('/tank_control', String, ir_sensor.tank_control_callback)
    
    print("IR Sensor Node is ready and waiting for commands")

    reading_count = 0
    
    while not rospy.is_shutdown():
        # Get simulated sensor reading
        status = ir_sensor.read_sensor()
        
        # Publish the status
        pub.publish(status)
        
        reading_count += 1
        
        # Only print when status changes and we're cleaning
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
