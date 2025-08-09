#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

"""ROS Node for Motor Control of Tank Cleaning Robot"""

"""change with actual GPIO control when available"""
LEFT_FORWARD = "LEFT_FORWARD_PIN"
LEFT_BACKWARD = "LEFT_BACKWARD_PIN" 
RIGHT_FORWARD = "RIGHT_FORWARD_PIN"
RIGHT_BACKWARD = "RIGHT_BACKWARD_PIN"

motor_state = {
    "left_motor": "STOPPED",
    "right_motor": "STOPPED",
    "last_command": "NONE"
}

is_active = False
last_printed_command = None

def print_motor_status():
    """Print current motor status in a visual way"""
    print(f"TANK MOTOR STATUS - Command: {motor_state['last_command']}")
    print(f"   Left Motor:  [{motor_state['left_motor']}]")
    print(f"   Right Motor: [{motor_state['right_motor']}]")
    
    if motor_state['last_command'] == "FORWARD":
        print("   Direction: MOVING FORWARD")
    elif motor_state['last_command'] == "REVERSE":
        print("   Direction: MOVING BACKWARD")
    elif motor_state['last_command'] == "TURN_LEFT":
        print("   Direction:   TURNING LEFT")
    elif motor_state['last_command'] == "TURN_RIGHT":
        print("   Direction:   TURNING RIGHT")
    elif motor_state['last_command'] == "PIVOT_LEFT":
        print("   Direction:  SHARP LEFT")
    elif motor_state['last_command'] == "PIVOT_RIGHT":
        print("   Direction:   SHARP RIGHT")
    elif motor_state['last_command'] == "STOP":
        print("   Direction:  STOPPED")

def motor_callback(msg):
    global is_active, last_printed_command
    command = msg.data
    motor_state["last_command"] = command

    if command == "STOP" and not is_active:
        motor_state["left_motor"] = "STOPPED"
        motor_state["right_motor"] = "STOPPED"
        return
    
    if command != "STOP":
        is_active = True
    if command != last_printed_command and is_active:
        print(f"Motor: {command}")
        last_printed_command = command

    motor_state["left_motor"] = "STOPPED"
    motor_state["right_motor"] = "STOPPED"

    if command == "FORWARD":
        motor_state["left_motor"] = "FORWARD"
        motor_state["right_motor"] = "FORWARD"
        
    elif command == "REVERSE":
        motor_state["left_motor"] = "REVERSE"
        motor_state["right_motor"] = "REVERSE"
        
    elif command == "TURN_LEFT":
        motor_state["left_motor"] = "REVERSE"
        motor_state["right_motor"] = "FORWARD"
        
    elif command == "TURN_RIGHT":
        motor_state["left_motor"] = "FORWARD" 
        motor_state["right_motor"] = "REVERSE"
        
    elif command == "PIVOT_LEFT":
        motor_state["left_motor"] = "STOPPED"
        motor_state["right_motor"] = "FORWARD"
        
    elif command == "PIVOT_RIGHT":
        motor_state["left_motor"] = "FORWARD"
        motor_state["right_motor"] = "STOPPED"
        
    elif command == "STOP":
        motor_state["left_motor"] = "STOPPED"
        motor_state["right_motor"] = "STOPPED"

def motor_node():
    rospy.init_node('motor_node')
    
    rospy.Subscriber('motor_control', String, motor_callback)
    rospy.Subscriber('/motor_command', String, motor_callback)
    
    print("Motor Node is ready and waiting for commands")
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_node()
    except rospy.ROSInterruptException:
        pass
