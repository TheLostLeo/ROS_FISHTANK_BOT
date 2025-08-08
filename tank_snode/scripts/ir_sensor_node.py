#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

IR_PIN = 17 

def sensor_node():
    rospy.init_node('sensor_node')
    pub = rospy.Publisher('/edge_status', String, queue_size=10)
    rate = rospy.Rate(10)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN)

    while not rospy.is_shutdown():
        status = "EDGE" if GPIO.input(IR_PIN) == 0 else "SAFE"
        pub.publish(status)
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    sensor_node()
