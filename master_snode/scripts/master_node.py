#!/usr/bin/env python3
"""
Master Node for Tank Cleaning Robot System
This node acts as a master controller that sends WebSocket commands to tank_snode
and manages the overall system state.
"""

import rospy
from std_msgs.msg import String
import asyncio
import websockets
import threading
import json
from datetime import datetime
import time
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv('/home/tll/catkin_ws/src/.env')

class MasterNode:
    def __init__(self):
        rospy.init_node('master_node')
        
        # Create publishers for commands and status
        # These will be used to send commands to tank_snode and receive status updates
        self.command_pub = rospy.Publisher('/master_commands', String, queue_size=10)
        self.status_pub = rospy.Publisher('/master_status', String, queue_size=10)
        
        # Create subscribers for control commands

        rospy.Subscriber('/master_control', String, self.control_callback)
        
        # Setting for websocket connection and initial system state
        # This will connect to the all tank_snode WebSocket server
        ws_host = os.getenv('WEBSOCKET_HOST', 'localhost')
        ws_port = os.getenv('WEBSOCKET_PORT', '8765')
        self.tank_websocket_uri = f"ws://{ws_host}:{ws_port}"  # tank_snode WebSocket server
        self.websocket_connection = None
        self.is_connected = False
        self.connection_established = False
        self.tank_status = "unknown"
        self.tank_is_cleaning = False
        self.last_command = None
        self.command_history = []
        self.event_loop = None
        print("MASTER NODE: Initializing...")
        print("MASTER NODE: Attempting to connect to tank...")
        rospy.loginfo("Master Node initialized")
        
    def control_callback(self, msg):
        """Ros topic to handle the control commands"""
        command = msg.data.strip().upper()
        print(f"MASTER NODE: Received ROS command: {command}")
        rospy.loginfo(f"Received control command: {command}")
        
        if command in ["START", "STOP", "STATUS"]:
            if self.is_connected and self.event_loop:
                try:
                    asyncio.run_coroutine_threadsafe(
                        self.send_websocket_command(command.lower()), self.event_loop
                    )
                except Exception as e:
                    print(f"WARNING MASTER NODE: Error scheduling command: {e}")
            else:
                print("WARNING MASTER NODE: Not connected to tank - command ignored")
                rospy.logwarn("Cannot send command - not connected to tank")
        else:
            rospy.logwarn(f"Unknown command received: {command}")
            
    async def send_websocket_command(self, command):
        """Websocket command that sends commans to tank_bot"""
        if not self.is_connected or not self.websocket_connection:
            print("ERROR MASTER NODE: No WebSocket connection to tank")
            return None
        try:
            # JSON commmand format
            message = {
                "command": command,
                "timestamp": time.time(),
                "type": "command"
            }
            await self.websocket_connection.send(json.dumps(message))
            print(f"MASTER NODE: Sent '{command}' command to tank")
            
            # LOG For the command history
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            command_record = {
                "timestamp": timestamp,
                "command": command,
                "response": "Command sent - waiting for response"
            }
            
            self.command_history.append(command_record)
            self.last_command = command_record
            self.command_pub.publish(f"COMMAND_SENT: {command}")
            
            return {"result": "Command sent successfully"}
            
        except Exception as e:
            print(f"ERROR MASTER NODE: Error sending command: {e}")
            rospy.logerr(f"Error sending WebSocket command: {e}")
            self.is_connected = False
            self.websocket_connection = None
            return None
            
    async def connect_to_tank(self):
        """Establish WebSocket connection"""
        max_retries = 5
        retry_delay = 2
        
        for attempt in range(max_retries):
            try:
                print(f"MASTER NODE: Connection attempt {attempt + 1}/{max_retries}")
                print("MASTER NODE: Connecting to tank...")
                
                self.websocket_connection = await websockets.connect(self.tank_websocket_uri)
                self.is_connected = True
                
                print("MASTER NODE: WebSocket connection established")
                print("MASTER NODE: Waiting for tank initialization...")
                welcome_msg = await self.websocket_connection.recv()
                welcome_data = json.loads(welcome_msg)
                if welcome_data.get("type") == "connection_established":
                    self.connection_established = True
                    self.tank_status = welcome_data.get("tank_status", "ready")
                    print("MASTER NODE: Connection fully established!")
                    print(f"MASTER NODE: Tank status: {self.tank_status}")
                    print("MASTER NODE: System ready for commands")
                    rospy.loginfo("Successfully connected and initialized with tank")
                    return True
                else:
                    print("WARNING MASTER NODE: Unexpected welcome message")
                    
            except ConnectionRefusedError:
                print(f"MASTER NODE: Tank not ready yet, retrying in {retry_delay}s...")
                await asyncio.sleep(retry_delay)
            except Exception as e:
                print(f"ERROR MASTER NODE: Connection error: {e}")
                await asyncio.sleep(retry_delay)
                
        print("ERROR MASTER NODE: Failed to connect after all retries")
        rospy.logerr("Failed to connect to tank WebSocket server after multiple attempts")
        return False
            
    async def listen_for_tank_updates(self):
        """Message handler for tank updates"""
        try:
            while self.is_connected and self.websocket_connection:
                try:
                    message = await asyncio.wait_for(self.websocket_connection.recv(), timeout=1.0)
                    data = json.loads(message)
                    
                    message_type = data.get("type", "unknown")
                    
                    if message_type == "status_update":
                        self.tank_status = "cleaning" if data.get("is_cleaning") else "ready"
                        self.tank_is_cleaning = data.get("is_cleaning", False)
                        print(f"MASTER NODE: Tank update - {data.get('message')}")
                        
                    elif message_type == "command_response":
                        result = data.get('result', 'Success')
                        command = data.get('command', 'unknown')
                        print(f"MASTER NODE: Tank responded to '{command}': {result}")
                        
                        if self.last_command:
                            self.last_command['response'] = data
                        
                        if 'status' in data:
                            self.tank_status = data['status']
                        if 'is_cleaning' in data:
                            self.tank_is_cleaning = data['is_cleaning']
                            
                        if command == "start":
                            if "already running" in result.lower():
                                self.status_pub.publish(f"TANK_RESPONSE: Bot already running")
                            else:
                                self.status_pub.publish(f"TANK_RESPONSE: Bot is running")
                        elif command == "stop":
                            if "not running" in result.lower():
                                self.status_pub.publish(f"TANK_RESPONSE: Bot is not running")  
                            else:
                                self.status_pub.publish(f"TANK_RESPONSE: Bot stopped")
                        elif command == "status":
                            self.status_pub.publish(f"TANK_RESPONSE: {result}")
                        else:
                            self.status_pub.publish(f"TANK_RESPONSE: {result}")
                        
                    else:
                        print(f"MASTER NODE: Received message: {data.get('message', str(data))}")
                        
                except asyncio.TimeoutError:
                    continue  
                except websockets.exceptions.ConnectionClosed:
                    print("MASTER NODE: Tank disconnected")
                    self.is_connected = False
                    break
                    
        except Exception as e:
            print(f"ERROR MASTER NODE: Error listening for updates: {e}")
            self.is_connected = False
            
    def publish_status(self):
        """Publish current master node status"""
        status_msg = {
            "connected_to_tank": self.is_connected,
            "connection_established": self.connection_established,
            "tank_status": self.tank_status,
            "tank_is_cleaning": self.tank_is_cleaning,
            "last_command": self.last_command,
            "total_commands_sent": len(self.command_history)
        }
        
        self.status_pub.publish(json.dumps(status_msg))
    """Command methods for tank control"""
    async def start_tank_cleaning(self):
        await self.send_websocket_command("start")
        
    async def stop_tank_cleaning(self):
        await self.send_websocket_command("stop")
        
    async def get_tank_status(self):
        return await self.send_websocket_command("status")

async def main():
    master = MasterNode()
    master.event_loop = asyncio.get_running_loop()
    success = await master.connect_to_tank()
    if not success:
        print("ERROR MASTER NODE: Failed to initialize - continuing anyway for status publishing")
        print("MASTER NODE: Will attempt reconnection periodically")
    
    if master.is_connected:
        listen_task = asyncio.create_task(master.listen_for_tank_updates())
    print("MASTER NODE: Status monitoring active")
    print("MASTER NODE: Ready to receive commands via /master_control topic")
    print("MASTER NODE: Publishing status on /master_status topic")
    
    try:
        while not rospy.is_shutdown():
            master.publish_status()
            
            if not master.is_connected:
                print("MASTER NODE: Attempting periodic reconnection...")
                reconnect_success = await master.connect_to_tank()
                if reconnect_success:
                    listen_task = asyncio.create_task(master.listen_for_tank_updates())
                
            await asyncio.sleep(1)
            
    except KeyboardInterrupt:
        print("\nMASTER NODE: Shutting down...")
    except Exception as e:
        print(f"ERROR MASTER NODE: Error in main loop: {e}")
    finally:
        if master.is_connected and master.websocket_connection:
            await master.websocket_connection.close()
        print("MASTER NODE: Shutdown complete")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        print("MASTER NODE: ROS shutdown requested")
    except KeyboardInterrupt:
        print("\nMASTER NODE: Keyboard interrupt")
    except Exception as e:
        print(f"ERROR MASTER NODE: Fatal error: {e}")
    finally:
        print("MASTER NODE: Goodbye!")
