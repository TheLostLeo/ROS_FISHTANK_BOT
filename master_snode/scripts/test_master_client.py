#!/usr/bin/env python3
"""
Test client for Master Node
This script can be used to send commands to the master_snode for testing
"""

import rospy
from std_msgs.msg import String
import sys

def send_command_to_master(command):
    """Send a command to the master node via ROS topic"""
    rospy.init_node('master_test_client', anonymous=True)
    
    # Publisher to send commands to master
    command_pub = rospy.Publisher('/master_control', String, queue_size=10)
    
    # Subscriber to receive responses
    def response_callback(msg):
        print(f"Master response: {msg.data}")
    
    status_sub = rospy.Subscriber('/master_status', String, response_callback)
    command_sub = rospy.Subscriber('/master_commands', String, response_callback)
    
    # Wait a moment for connections to establish
    rospy.sleep(1)
    
    # Send the command
    print(f"Sending command to master: {command}")
    command_pub.publish(command)
    
    # Wait a moment to receive response
    rospy.sleep(2)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python3 test_master_client.py <command>")
        print("Commands: start, stop, status")
        sys.exit(1)
    
    command = sys.argv[1]
    
    if command.lower() not in ['start', 'stop', 'status']:
        print("Invalid command. Use: start, stop, or status")
        sys.exit(1)
    
    try:
        send_command_to_master(command.upper())
        print("Command sent successfully")
    except rospy.ROSInterruptException:
        print("Test client interrupted")
    except Exception as e:
        print(f"Error: {e}")
