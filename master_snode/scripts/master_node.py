#!/usr/bin/env python3
"""
Master Node for Tank Cleaning Robot System
This node acts as a master controller that sends WebSocket commands to tank_snode
"""

import rospy
from std_msgs.msg import String
import asyncio
import websockets
import threading
import json
from datetime import datetime

class MasterNode:
    def __init__(self):
        rospy.init_node('master_node')
        
        # Publishers
        self.command_pub = rospy.Publisher('/master_commands', String, queue_size=10)
        self.status_pub = rospy.Publisher('/master_status', String, queue_size=10)
        
        # Subscribers - will be used to receive commands from other nodes or topics
        rospy.Subscriber('/master_control', String, self.control_callback)
        
        # WebSocket connection settings
        self.tank_websocket_uri = "ws://localhost:8765"  # tank_snode WebSocket server
        self.websocket_connection = None
        self.is_connected = False
        
        # System state
        self.tank_status = "unknown"
        self.last_command = None
        self.command_history = []
        
        rospy.loginfo("Master Node initialized")
        
    def control_callback(self, msg):
        """Handle control commands received via ROS topics"""
        command = msg.data.strip().upper()
        rospy.loginfo(f"Received control command: {command}")
        
        if command in ["START", "STOP", "STATUS"]:
            asyncio.run(self.send_websocket_command(command.lower()))
        else:
            rospy.logwarn(f"Unknown command received: {command}")
            
    async def send_websocket_command(self, command):
        """Send command to tank_snode via WebSocket"""
        try:
            if not self.is_connected:
                await self.connect_to_tank()
                
            if self.websocket_connection:
                await self.websocket_connection.send(command)
                response = await self.websocket_connection.recv()
                
                # Log the command and response
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                command_record = {
                    "timestamp": timestamp,
                    "command": command,
                    "response": response
                }
                
                self.command_history.append(command_record)
                self.last_command = command_record
                
                rospy.loginfo(f"Sent command '{command}' to tank, received: {response}")
                
                # Publish the response
                self.command_pub.publish(f"COMMAND_SENT: {command}")
                self.status_pub.publish(f"TANK_RESPONSE: {response}")
                
                return response
            else:
                rospy.logerr("No WebSocket connection to tank")
                return None
                
        except Exception as e:
            rospy.logerr(f"Error sending WebSocket command: {e}")
            self.is_connected = False
            self.websocket_connection = None
            return None
            
    async def connect_to_tank(self):
        """Establish WebSocket connection to tank_snode"""
        try:
            rospy.loginfo(f"Connecting to tank at {self.tank_websocket_uri}")
            self.websocket_connection = await websockets.connect(self.tank_websocket_uri)
            self.is_connected = True
            rospy.loginfo("Successfully connected to tank WebSocket server")
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to tank WebSocket server: {e}")
            self.is_connected = False
            self.websocket_connection = None
            
    def publish_status(self):
        """Publish current master node status"""
        status_msg = {
            "connected_to_tank": self.is_connected,
            "tank_status": self.tank_status,
            "last_command": self.last_command,
            "total_commands_sent": len(self.command_history)
        }
        
        self.status_pub.publish(json.dumps(status_msg))
        
    def start_tank_cleaning(self):
        """Convenience method to start tank cleaning"""
        asyncio.run(self.send_websocket_command("start"))
        
    def stop_tank_cleaning(self):
        """Convenience method to stop tank cleaning"""
        asyncio.run(self.send_websocket_command("stop"))
        
    def get_tank_status(self):
        """Convenience method to get tank status"""
        return asyncio.run(self.send_websocket_command("status"))

def main():
    try:
        master = MasterNode()
        
        # Publish status periodically
        rate = rospy.Rate(0.5)  # 0.5 Hz (every 2 seconds)
        
        while not rospy.is_shutdown():
            master.publish_status()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Master node shutting down")
    except Exception as e:
        rospy.logerr(f"Master node error: {e}")

if __name__ == '__main__':
    main()
