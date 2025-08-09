#!/usr/bin/env python3
"""Decision Node for Tank Cleaning Robot
This node handles decision-making based on edge detection from the IR sensor
and controls the motor node 
"""
import rospy
from std_msgs.msg import String
import time
import random

motor_pub = None
comm_pub = None
is_cleaning = False

current_direction = "RIGHT"  
row_completed = False
going_up = True
cleaning_pattern_active = False

move_counter = 0
rows_completed = 0

def print_cleaning_status():
    """Print visual status of cleaning operation"""
    global move_counter, rows_completed
    print("TANK CLEANING ROBOT - DEMO MODE")
    print(f"Status: {'CLEANING' if is_cleaning else 'STANDBY'}")
    print(f"Current Direction: {current_direction}")
    print(f"Moves Made: {move_counter}")
    print(f"Rows Completed: {rows_completed}")

def tank_control_callback(msg):
    """Handle tank control commands from comm_node"""
    global is_cleaning, current_direction, cleaning_pattern_active, going_up, move_counter, rows_completed
    command = msg.data
    
    if command == "START_CLEANING":
        is_cleaning = True
        cleaning_pattern_active = True
        current_direction = "RIGHT" 
        going_up = True
        move_counter = 0
        rows_completed = 0
        
        print_cleaning_status()
        
        rospy.loginfo("Starting tank cleaning operation with zig-zag pattern")
        comm_pub.publish("Tank cleaning operation started - zig-zag pattern")
        motor_pub.publish("FORWARD")  # Start moving in current direction
        
    elif command == "STOP_CLEANING":
        is_cleaning = False
        cleaning_pattern_active = False
        rospy.loginfo("Stopping tank cleaning operation")
        motor_pub.publish("STOP")
        comm_pub.publish("Tank cleaning operation stopped")
"""current algorithm is a zig-zag pattern with turns at edges"""
def execute_zigzag_turn():
    global current_direction, going_up
    
    print(f"Decision: Turning from {current_direction} to new direction")
    
    motor_pub.publish("STOP")
    time.sleep(0.5)
    
    if current_direction == "RIGHT":
        if going_up:
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_LEFT")
            time.sleep(1.5)  # Longer turn for 90 degree
            current_direction = "UP"
        else: 
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_LEFT")
            time.sleep(1.5)
            current_direction = "DOWN"
            
    elif current_direction == "LEFT":
        if going_up:
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_RIGHT")
            time.sleep(1.5)
            current_direction = "UP"
        else:
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_RIGHT")
            time.sleep(1.5)
            current_direction = "DOWN"
            
    elif current_direction == "UP":
        motor_pub.publish("REVERSE")
        time.sleep(0.5)
        
        turn_direction = random.choice(["TURN_LEFT", "TURN_RIGHT"])
        motor_pub.publish(turn_direction)
        time.sleep(1.5)
        
        current_direction = "LEFT" if turn_direction == "TURN_LEFT" else "RIGHT"
        going_up = False  
    elif current_direction == "DOWN":
        motor_pub.publish("REVERSE")
        time.sleep(0.5)
        turn_direction = random.choice(["TURN_LEFT", "TURN_RIGHT"])
        motor_pub.publish(turn_direction)
        time.sleep(1.5)

        current_direction = "LEFT" if turn_direction == "TURN_LEFT" else "RIGHT"
        going_up = True 
    
    motor_pub.publish("FORWARD")
    print(f"Decision: Now moving {current_direction}")

def edge_callback(msg):
    """Handle edge detection"""
    global is_cleaning, cleaning_pattern_active, move_counter, rows_completed
    status = msg.data

    if not is_cleaning:
        motor_pub.publish("STOP")
        return

    if status == "EDGE":
        move_counter += 1
        
        if cleaning_pattern_active:
            print("Decision: Edge detected")
            
            rospy.loginfo("Edge detected")
            execute_zigzag_turn()
            
            if current_direction in ["LEFT", "RIGHT"]:
                rows_completed += 1
                print(f"Decision: Row {rows_completed} completed")
            
        else:
            print("Decision: Using fallback avoidance pattern")
            rospy.loginfo("Edge detected during cleaning! Executing avoidance maneuver.")
            comm_pub.publish("Edge detected! Executing avoidance maneuver.")

            motor_pub.publish("STOP")
            time.sleep(0.5)

            motor_pub.publish("REVERSE")
            time.sleep(1)

            turn_direction = random.choice(["TURN_LEFT", "TURN_RIGHT"])
            motor_pub.publish(turn_direction)
            time.sleep(1)

            motor_pub.publish("FORWARD")
            comm_pub.publish("Avoided edge. Resuming cleaning.")
    else:
        motor_pub.publish("FORWARD")

def decision_node():
    global motor_pub, comm_pub
    rospy.init_node('decision_node')

    motor_pub = rospy.Publisher('/motor_command', String, queue_size=10)
    comm_pub = rospy.Publisher('/status_report', String, queue_size=10)

    rospy.Subscriber('/edge_status', String, edge_callback)
    rospy.Subscriber('/tank_control', String, tank_control_callback)
    
    print("Decision Node is ready and waiting for commands")
    rospy.spin()

if __name__ == '__main__':
    decision_node()
