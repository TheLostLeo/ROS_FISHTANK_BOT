#!/usr/bin/env python3
"""
Demo Water Pump Controller Node for Tank Cleaning Robot System
This is a demonstration version that simulates water pump operations without actual hardware
"""

import rospy
from std_msgs.msg import String, Bool
import time
import threading

# Demo pump states - simulating actual hardware
pump_state = {
    "running": False,
    "operation": "idle",  # idle, draining, filling, cleaning
    "flow_rate": 0,      # liters per minute
    "water_level": True,  # True = OK, False = LOW
    "last_command": "NONE",
    "total_runtime": 0,   # seconds
    "cycles_completed": 0
}

# State tracking for publishing only when changes occur
last_published_state = {
    "running": None,
    "operation": None,
    "flow_rate": None,
    "water_level": None,
    "cycles_completed": None
}

is_active = False
last_printed_command = None
pump_thread = None
pump_stop_event = threading.Event()

def print_pump_status():
    """Print current pump status in a visual way"""
    print(f"DEMO PUMP STATUS - Command: {pump_state['last_command']}")
    print(f"   Operation:   [{pump_state['operation'].upper()}]")
    print(f"   Pump State:  [{'RUNNING' if pump_state['running'] else 'STOPPED'}]")
    print(f"   Flow Rate:   {pump_state['flow_rate']} L/min")
    print(f"   Water Level: {'OK' if pump_state['water_level'] else 'LOW'}")
    print(f"   Runtime:     {pump_state['total_runtime']} seconds")
    
    if pump_state['operation'] == "draining":
        print("   Action: REMOVING DIRTY WATER")
    elif pump_state['operation'] == "filling":
        print("   Action: ADDING FRESH WATER")
    elif pump_state['operation'] == "cleaning":
        print("   Action: FULL WATER CHANGE CYCLE")
    elif pump_state['operation'] == "idle":
        print("   Action: STANDBY")

def simulate_pump_operation(operation, duration):
    """Simulate pump operation for demo purposes"""
    global pump_state, pump_stop_event, pump_response_pub
    
    pump_state["running"] = True
    pump_state["operation"] = operation
    
    # Set flow rate based on operation
    if operation == "draining":
        pump_state["flow_rate"] = 2.5  # L/min
    elif operation == "filling":
        pump_state["flow_rate"] = 1.8  # L/min
    elif operation == "cleaning":
        pump_state["flow_rate"] = 2.0  # L/min
    else:
        pump_state["flow_rate"] = 0
    
    start_time = time.time()
    
    # Send operation start message to master
    pump_response_pub.publish(f"PUMP_STARTED:{operation}:{duration}s")
    
    # Simulate operation progress
    while not pump_stop_event.is_set() and (time.time() - start_time) < duration:
        elapsed = int(time.time() - start_time)
        pump_state["total_runtime"] += 1
        
        # Simulate water level changes during operation
        if operation == "draining" and elapsed > 5:
            pump_state["water_level"] = False  # Water getting low
        elif operation == "filling" and elapsed > 3:
            pump_state["water_level"] = True   # Water level restored
            
        time.sleep(1)
    
    # Operation complete
    pump_state["running"] = False
    pump_state["operation"] = "idle"
    pump_state["flow_rate"] = 0
    pump_state["cycles_completed"] += 1
    
    # Send completion/stop message to master
    if not pump_stop_event.is_set():
        pump_response_pub.publish(f"PUMP_COMPLETED:{operation}:success")
    else:
        pump_response_pub.publish(f"PUMP_STOPPED:{operation}:user_request")
    
    pump_stop_event.clear()

def pump_control_callback(msg):
    """Handle pump control commands from master"""
    global is_active, last_printed_command, pump_thread, pump_stop_event, pump_response_pub
    
    command = msg.data.strip().upper()
    pump_state["last_command"] = command
    
    # Only process when command changes to avoid spam
    if command != last_printed_command:
        last_printed_command = command
        is_active = True
        
        # Stop any running operation first
        if pump_thread and pump_thread.is_alive():
            pump_stop_event.set()
            pump_thread.join(timeout=2)
        
        if command == "DRAIN":
            pump_response_pub.publish("PUMP_COMMAND_RECEIVED:DRAIN:starting")
            pump_thread = threading.Thread(target=simulate_pump_operation, args=("draining", 15))
            pump_thread.start()
            
        elif command == "FILL":
            pump_response_pub.publish("PUMP_COMMAND_RECEIVED:FILL:starting")
            pump_thread = threading.Thread(target=simulate_pump_operation, args=("filling", 12))
            pump_thread.start()
            
        elif command == "CLEAN":
            # Full water change cycle - drain then fill
            pump_response_pub.publish("PUMP_COMMAND_RECEIVED:WATER_CHANGE:starting_cycle")
            pump_thread = threading.Thread(target=water_change_cycle)
            pump_thread.start()
            
        elif command == "STOP":
            pump_stop_event.set()
            pump_state["running"] = False
            pump_state["operation"] = "idle"
            pump_state["flow_rate"] = 0
            pump_response_pub.publish("PUMP_COMMAND_RECEIVED:STOP:all_operations_stopped")

def water_change_cycle():
    """Simulate a complete water change cycle"""
    global pump_response_pub
    
    # Send cycle start message
    pump_response_pub.publish("PUMP_CYCLE_PHASE:water_change:drain_phase_starting")
    simulate_pump_operation("draining", 15)
    
    if not pump_stop_event.is_set():
        pump_response_pub.publish("PUMP_CYCLE_PHASE:water_change:fill_phase_starting")
        time.sleep(2)  # Brief pause between operations
        simulate_pump_operation("filling", 12)
    
    if not pump_stop_event.is_set():
        pump_state["water_level"] = True  # Fresh water, good level
        pump_response_pub.publish("PUMP_CYCLE_COMPLETED:water_change:fresh_water_ready")

def master_control_callback(msg):
    """Handle direct commands from master_control (legacy compatibility)"""
    command = msg.data.strip().upper()
    
    # Map master commands to pump commands
    if command in ["DRAIN", "FILL", "WATER_CHANGE", "PUMP_STOP"]:
        if command == "WATER_CHANGE":
            pump_control_callback(String("CLEAN"))
        elif command == "PUMP_STOP":
            pump_control_callback(String("STOP"))
        else:
            pump_control_callback(String(command))

def publish_status():
    """Publish pump status to ROS topics - only when there are changes"""
    global last_published_state
    
    # Check if any important state has changed
    state_changed = (
        pump_state['running'] != last_published_state['running'] or
        pump_state['operation'] != last_published_state['operation'] or
        pump_state['flow_rate'] != last_published_state['flow_rate'] or
        pump_state['water_level'] != last_published_state['water_level'] or
        pump_state['cycles_completed'] != last_published_state['cycles_completed']
    )
    
    # Only publish if something actually changed
    if state_changed:
        # Status message
        status_msg = (f"Running={pump_state['running']}, "
                     f"Operation={pump_state['operation']}, "
                     f"FlowRate={pump_state['flow_rate']}, "
                     f"Runtime={pump_state['total_runtime']}, "
                     f"Cycles={pump_state['cycles_completed']}")
        
        status_pub.publish(status_msg)
        
        # Water level
        water_level_pub.publish(pump_state['water_level'])
        
        # Flow rate
        flow_msg = f"Current: {pump_state['flow_rate']} L/min"
        flow_rate_pub.publish(flow_msg)
        
        # Update the last published state
        last_published_state.update({
            'running': pump_state['running'],
            'operation': pump_state['operation'],
            'flow_rate': pump_state['flow_rate'],
            'water_level': pump_state['water_level'],
            'cycles_completed': pump_state['cycles_completed']
        })

def main():
    global status_pub, water_level_pub, flow_rate_pub, pump_response_pub
    
    rospy.init_node('demo_water_pump_controller')
    
    # Publishers
    status_pub = rospy.Publisher('/pump_status', String, queue_size=10)
    water_level_pub = rospy.Publisher('/water_level', Bool, queue_size=10)
    flow_rate_pub = rospy.Publisher('/flow_rate', String, queue_size=10)
    pump_response_pub = rospy.Publisher('/pump_response', String, queue_size=10)  # New response publisher
    
    # Subscribers
    rospy.Subscriber('/pump_control', String, pump_control_callback)
    rospy.Subscriber('/master_control', String, master_control_callback)
    
    print("DEMO PUMP CONTROLLER: Water pump controller initialized")
    print("DEMO PUMP CONTROLLER: Ready to receive commands from master")
    
    # Main loop - publish status regularly
    rate = rospy.Rate(2)  # 2 Hz status updates
    while not rospy.is_shutdown():
        publish_status()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("DEMO PUMP CONTROLLER: ROS shutdown requested")
    except KeyboardInterrupt:
        print("\nDEMO PUMP CONTROLLER: Keyboard interrupt received")
    finally:
        # Clean shutdown - stop any running pump operations
        if 'pump_thread' in globals() and pump_thread and pump_thread.is_alive():
            pump_stop_event.set()
            pump_thread.join(timeout=3)
        print("DEMO PUMP CONTROLLER: Shutdown complete")
