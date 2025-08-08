#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

# Motor control pins (adjust as needed)
LEFT_FORWARD = 22
LEFT_BACKWARD = 27
RIGHT_FORWARD = 23
RIGHT_BACKWARD = 24

def motor_callback(msg):
    command = msg.data
    rospy.loginfo(f"Motor command: {command}")

    # Stop all motors first
    GPIO.output(LEFT_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_BACKWARD, GPIO.LOW)

    if command == "FORWARD":
        GPIO.output(LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(RIGHT_FORWARD, GPIO.HIGH)
    elif command == "REVERSE":
        GPIO.output(LEFT_BACKWARD, GPIO.HIGH)
        GPIO.output(RIGHT_BACKWARD, GPIO.HIGH)
    elif command == "TURN_LEFT":
        # Turn left: right wheel forward, left wheel backward
        GPIO.output(RIGHT_FORWARD, GPIO.HIGH)
        GPIO.output(LEFT_BACKWARD, GPIO.HIGH)
    elif command == "TURN_RIGHT":
        # Turn right: left wheel forward, right wheel backward
        GPIO.output(LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(RIGHT_BACKWARD, GPIO.HIGH)
    elif command == "PIVOT_LEFT":
        # Sharper left turn: right wheel forward only
        GPIO.output(RIGHT_FORWARD, GPIO.HIGH)
    elif command == "PIVOT_RIGHT":
        # Sharper right turn: left wheel forward only
        GPIO.output(LEFT_FORWARD, GPIO.HIGH)
    elif command == "STOP":
        pass  # Already stopped

def motor_node():
    rospy.init_node('motor_node')
    GPIO.setmode(GPIO.BCM)
    for pin in [LEFT_FORWARD, LEFT_BACKWARD, RIGHT_FORWARD, RIGHT_BACKWARD]:
        GPIO.setup(pin, GPIO.OUT)

    rospy.Subscriber('/motor_command', String, motor_callback)
    rospy.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    motor_node()
