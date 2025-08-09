#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time
import random

motor_pub = None
comm_pub = None
is_cleaning = False

# Zig-zag pattern variables
current_direction = "RIGHT"  # Start moving right from left bottom
row_completed = False
going_up = True
cleaning_pattern_active = False

# Demo visualization
move_counter = 0
rows_completed = 0

def print_cleaning_status():
    """Print visual status of cleaning operation"""
    global move_counter, rows_completed
    print("\n" + "="*60)
    print("TANK CLEANING ROBOT - DEMO MODE")
    print(f"Status: {'CLEANING' if is_cleaning else 'STANDBY'}")
    print(f"Current Direction: {current_direction}")
    print(f"Moves Made: {move_counter}")
    print(f"Rows Completed: {rows_completed}")
    print("="*60)

def tank_control_callback(msg):
    """Handle tank control commands from comm_node"""
    global is_cleaning, current_direction, cleaning_pattern_active, going_up, move_counter, rows_completed
    command = msg.data
    
    if command == "START_CLEANING":
        is_cleaning = True
        cleaning_pattern_active = True
        current_direction = "RIGHT"  # Start by moving right
        going_up = True
        move_counter = 0
        rows_completed = 0
        
        print("STARTING TANK CLEANING OPERATION!")
        print("Implementing zig-zag cleaning pattern")
        print("Pattern: RIGHT → UP → LEFT → UP → RIGHT → UP...")
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

def execute_zigzag_turn():
    """Execute the zig-zag pattern turn sequence"""
    global current_direction, going_up
    
    print(f"Decision: Turning from {current_direction} to new direction")
    
    # Stop current movement
    motor_pub.publish("STOP")
    time.sleep(0.5)
    
    if current_direction == "RIGHT":
        # Hit right edge, turn around to go left
        if going_up:
            # Turn up-left: reverse, turn left, then forward
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_LEFT")
            time.sleep(1.5)  # Longer turn for 90 degree
            current_direction = "UP"
        else:
            # Turn down-left: reverse, turn left, then forward  
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_LEFT")
            time.sleep(1.5)
            current_direction = "DOWN"
            
    elif current_direction == "LEFT":
        # Hit left edge, turn around to go right
        if going_up:
            # Turn up-right: reverse, turn right, then forward
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_RIGHT")
            time.sleep(1.5)
            current_direction = "UP"
        else:
            # Turn down-right: reverse, turn right, then forward
            motor_pub.publish("REVERSE")
            time.sleep(0.5)
            motor_pub.publish("TURN_RIGHT")
            time.sleep(1.5)
            current_direction = "DOWN"
            
    elif current_direction == "UP":
        # Hit top edge, now go across (left or right)
        motor_pub.publish("REVERSE")
        time.sleep(0.5)
        
        # Determine which way to turn based on last horizontal direction
        turn_direction = random.choice(["TURN_LEFT", "TURN_RIGHT"])
        motor_pub.publish(turn_direction)
        time.sleep(1.5)
        
        # Now moving horizontally at top
        current_direction = "LEFT" if turn_direction == "TURN_LEFT" else "RIGHT"
        going_up = False  # Now we'll be working our way down
        
    elif current_direction == "DOWN":
        # Hit bottom edge, now go across (left or right)
        motor_pub.publish("REVERSE")
        time.sleep(0.5)
        
        # Determine which way to turn
        turn_direction = random.choice(["TURN_LEFT", "TURN_RIGHT"])
        motor_pub.publish(turn_direction)
        time.sleep(1.5)
        
        # Now moving horizontally at bottom
        current_direction = "LEFT" if turn_direction == "TURN_LEFT" else "RIGHT"
        going_up = True  # Now we'll be working our way up
    
    # Resume forward movement in new direction
    motor_pub.publish("FORWARD")
    print(f"Decision: Now moving {current_direction}")

def edge_callback(msg):
    """Handle edge detection - execute zig-zag pattern when cleaning"""
    global is_cleaning, cleaning_pattern_active, move_counter, rows_completed
    status = msg.data

    if not is_cleaning:
        # If not cleaning, just stop and wait - no printing
        motor_pub.publish("STOP")
        return

    if status == "EDGE":
        move_counter += 1
        
        if cleaning_pattern_active:
            print("Decision: Edge detected - executing zig-zag turn")
            
            rospy.loginfo("Edge detected - executing zig-zag pattern turn")
            execute_zigzag_turn()
            
            # Check if we completed a row
            if current_direction in ["LEFT", "RIGHT"]:
                rows_completed += 1
                print(f"Decision: Row {rows_completed} completed")
            
        else:
            # Fallback to original random avoidance if pattern fails
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
        # Safe area - continue cleaning by moving forward
        motor_pub.publish("FORWARD")

def decision_node():
    global motor_pub, comm_pub
    rospy.init_node('decision_node')

    motor_pub = rospy.Publisher('/motor_command', String, queue_size=10)
    comm_pub = rospy.Publisher('/status_report', String, queue_size=10)

    # Subscribe to edge status from IR sensor
    rospy.Subscriber('/edge_status', String, edge_callback)
    
    # Subscribe to tank control commands from comm_node
    rospy.Subscriber('/tank_control', String, tank_control_callback)
    
    print("Decision Node is ready and waiting for commands")
    rospy.spin()

if __name__ == '__main__':
    decision_node()
