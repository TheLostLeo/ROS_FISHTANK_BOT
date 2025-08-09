#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

# DEMO MODE: Simulating motor control without GPIO
# This version uses print statements to show motor actions for demonstration

# Motor control simulation - no GPIO needed
LEFT_FORWARD = "LEFT_FORWARD_PIN"
LEFT_BACKWARD = "LEFT_BACKWARD_PIN" 
RIGHT_FORWARD = "RIGHT_FORWARD_PIN"
RIGHT_BACKWARD = "RIGHT_BACKWARD_PIN"

# Motor state tracking for visualization
motor_state = {
    "left_motor": "STOPPED",
    "right_motor": "STOPPED",
    "last_command": "NONE"
}

# Track if we're in active cleaning mode and last printed command
is_active = False
last_printed_command = None

def print_motor_status():
    """Print current motor status in a visual way"""
    print("=" * 50)
    print(f"TANK MOTOR STATUS - Command: {motor_state['last_command']}")
    print(f"   Left Motor:  [{motor_state['left_motor']}]")
    print(f"   Right Motor: [{motor_state['right_motor']}]")
    
    # Visual representation
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
    
    print("=" * 50)

def motor_callback(msg):
    global is_active, last_printed_command
    command = msg.data
    
    # Update motor state tracking
    motor_state["last_command"] = command

    # Only print when command changes and we're in active mode
    if command == "STOP" and not is_active:
        # Silent stop when not active - just update state
        motor_state["left_motor"] = "STOPPED"
        motor_state["right_motor"] = "STOPPED"
        return
    
    # Mark as active when we receive non-stop commands
    if command != "STOP":
        is_active = True
    
    # Only print when command actually changes
    if command != last_printed_command and is_active:
        print(f"Motor: {command}")
        last_printed_command = command
    
    # Update motor state silently

    # Update motor state silently
    motor_state["left_motor"] = "STOPPED"
    motor_state["right_motor"] = "STOPPED"

    # Simulate motor control with state updates
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
