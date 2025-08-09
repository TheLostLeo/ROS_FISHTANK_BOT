#!/usr/bin/env python3
"""
Master Brain Node for Tank Cleaning Robot System
This node acts as the central coordinator that manages all subsystems through ROS topics.
It receives user commands and coordinates tank cleaning, water pump, and other operations.
"""

import rospy
from std_msgs.msg import String, Bool
import json
from datetime import datetime
import time

class MasterBrain:
    def __init__(self):
        rospy.init_node('master_brain')
        
        # Publishers - send commands to subsystems
        self.bot_command_pub = rospy.Publisher('/bot_commands', String, queue_size=10)
        self.pump_command_pub = rospy.Publisher('/pump_control', String, queue_size=10)
        self.system_status_pub = rospy.Publisher('/master_status', String, queue_size=10)
        self.master_response_pub = rospy.Publisher('/master_commands', String, queue_size=10)
        
        # Subscribers - receive status from subsystems and user commands
        rospy.Subscriber('/master_control', String, self.user_command_callback)
        rospy.Subscriber('/tank_response', String, self.tank_response_callback)
        rospy.Subscriber('/tank_status', String, self.tank_status_callback)
        rospy.Subscriber('/bot_connection', String, self.bot_connection_callback)
        rospy.Subscriber('/pump_status', String, self.pump_status_callback)
        rospy.Subscriber('/pump_response', String, self.pump_response_callback)  # New pump response subscriber
        rospy.Subscriber('/water_level', Bool, self.water_level_callback)
        
        # System state variables
        self.bot_connected = False
        self.tank_status = "unknown"
        self.tank_is_cleaning = False
        self.pump_running = False
        self.pump_operation = "idle"
        self.water_level_ok = True
        
        # Warning state tracking to prevent repeated warnings
        self.warned_pump_tank_simultaneous = False
        self.warned_low_water_cleaning = False
        
        # Command history and coordination
        self.last_user_command = None
        self.command_history = []
        self.system_operations = []  # Track multi-step operations
        
        print("MASTER: Central coordinator initialized")
        print("MASTER: Managing tank, pump, and user interface")
        rospy.loginfo("Master Node started - coordinating all subsystems")

    def user_command_callback(self, msg):
        """Handle commands from user interface (test_user_input.py)"""
        command = msg.data.strip().upper()
        print(f"MASTER: Received user command: {command}")
        
        # Log command
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.last_user_command = {
            "command": command,
            "timestamp": timestamp,
            "status": "processing"
        }
        self.command_history.append(self.last_user_command)
        
        # Process user commands
        if command in ["START", "STOP", "STATUS"]:
            self.handle_tank_command(command)
        elif command in ["DRAIN", "FILL", "WATER_CHANGE", "PUMP_STOP"]:
            self.handle_pump_command(command)
        elif command == "SYSTEM_STATUS":
            self.publish_system_status(print_status=True)  # Print when user requests it
        else:
            print(f"MASTER: Unknown command: {command}")
            self.master_response_pub.publish(f"ERROR: Unknown command '{command}'")
            
    def handle_tank_command(self, command):
        """Handle tank cleaning related commands"""
        print(f"MASTER: Processing tank command: {command}")
        
        if not self.bot_connected:
            print("MASTER: Bot controller not connected - cannot execute tank command")
            self.master_response_pub.publish("ERROR: Tank not connected")
            return
            
        if command == "START":
            if self.tank_is_cleaning:
                self.master_response_pub.publish("TANK_RESPONSE: Bot already running")
            else:
                print("MASTER: Sending START command to tank")
                self.bot_command_pub.publish("start")
        elif command == "STOP":
            if not self.tank_is_cleaning:
                self.master_response_pub.publish("TANK_RESPONSE: Bot is not running")
            else:
                print("MASTER: Sending STOP command to tank")
                self.bot_command_pub.publish("stop")
        elif command == "STATUS":
            print("MASTER: Requesting tank status")
            self.bot_command_pub.publish("status")
            
    def handle_pump_command(self, command):
        """Handle water pump related commands"""
        print(f"MASTER: Processing pump command: {command}")
        
        if command == "DRAIN":
            print("MASTER: Starting tank drain operation")
            self.pump_command_pub.publish("DRAIN")
            self.master_response_pub.publish("PUMP_RESPONSE: Starting drain operation")
        elif command == "FILL":
            print("MASTER: Starting tank fill operation")
            self.pump_command_pub.publish("FILL")
            self.master_response_pub.publish("PUMP_RESPONSE: Starting fill operation")
        elif command == "WATER_CHANGE":
            print("MASTER: Starting complete water change cycle")
            self.pump_command_pub.publish("CLEAN")
            self.master_response_pub.publish("PUMP_RESPONSE: Starting water change cycle")
        elif command == "PUMP_STOP":
            print("MASTER: Stopping pump operations")
            self.pump_command_pub.publish("STOP")
            self.master_response_pub.publish("PUMP_RESPONSE: Stopping pump")
            
    def tank_response_callback(self, msg):
        """Handle responses from tank via bot controller"""
        response = msg.data
        print(f"MASTER: Tank response: {response}")
        
        # Parse response format: "RESPONSE:command:result:status"
        if response.startswith("RESPONSE:"):
            parts = response.split(":", 3)
            if len(parts) >= 4:
                _, command, result, status = parts
                
                if command == "start":
                    if "already" in result.lower():
                        self.master_response_pub.publish("TANK_RESPONSE: Bot already running")
                    else:
                        self.master_response_pub.publish("TANK_RESPONSE: Bot is running")
                elif command == "stop":
                    if "not running" in result.lower():
                        self.master_response_pub.publish("TANK_RESPONSE: Bot is not running")
                    else:
                        self.master_response_pub.publish("TANK_RESPONSE: Bot stopped")
                elif command == "status":
                    self.master_response_pub.publish(f"TANK_RESPONSE: {result}")
        elif response.startswith("ERROR:"):
            self.master_response_pub.publish(f"TANK_RESPONSE: {response}")
            
    def tank_status_callback(self, msg):
        """Handle tank status updates"""
        status_msg = msg.data
        
        if status_msg.startswith("STATUS:"):
            status = status_msg.replace("STATUS:", "").strip()
            
            # Update tank state
            prev_cleaning = self.tank_is_cleaning
            self.tank_status = status
            self.tank_is_cleaning = (status == "cleaning")
            
            if prev_cleaning != self.tank_is_cleaning:
                print(f"MASTER: Tank cleaning state changed: {status}")
                
    def bot_connection_callback(self, msg):
        """Handle bot controller connection status"""
        connection_msg = msg.data
        
        if connection_msg.startswith("CONNECTED:"):
            if not self.bot_connected:
                self.bot_connected = True
                print("MASTER: Bot controller connected to tank")
        elif connection_msg.startswith("DISCONNECTED:") or connection_msg.startswith("FAILED:"):
            if self.bot_connected:
                self.bot_connected = False
                print("MASTER: Bot controller disconnected from tank")

    def pump_status_callback(self, msg):
        """Handle pump status updates"""
        status_msg = msg.data
        
        # Parse pump status for state tracking (don't print routine updates)
        prev_running = self.pump_running
        prev_operation = self.pump_operation
        
        if "Running=True" in status_msg:
            self.pump_running = True
        elif "Running=False" in status_msg:
            self.pump_running = False
            
        if "Operation=" in status_msg:
            try:
                operation_part = status_msg.split("Operation=")[1].split(",")[0]
                self.pump_operation = operation_part.strip()
            except:
                pass
        
        # Only print when pump state actually changes
        if prev_running != self.pump_running or prev_operation != self.pump_operation:
            print(f"MASTER: Pump state changed - Running: {self.pump_running}, Operation: {self.pump_operation}")
    
    def pump_response_callback(self, msg):
        """Handle pump response/feedback messages"""
        response = msg.data
        
        # Parse and display pump response messages
        if response.startswith("PUMP_COMMAND_RECEIVED:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, command, status = parts
                print(f"MASTER: Pump acknowledged {command} command - {status}")
                self.master_response_pub.publish(f"PUMP_RESPONSE: {command} {status}")
                
        elif response.startswith("PUMP_STARTED:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, operation, duration = parts
                print(f"MASTER: Pump started {operation} operation ({duration})")
                self.master_response_pub.publish(f"PUMP_RESPONSE: Started {operation} operation")
                
        elif response.startswith("PUMP_COMPLETED:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, operation, result = parts
                print(f"MASTER: Pump completed {operation} - {result}")
                self.master_response_pub.publish(f"PUMP_RESPONSE: {operation.capitalize()} completed successfully")
                
        elif response.startswith("PUMP_STOPPED:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, operation, reason = parts
                print(f"MASTER: Pump stopped {operation} - {reason}")
                self.master_response_pub.publish(f"PUMP_RESPONSE: {operation.capitalize()} stopped")
                
        elif response.startswith("PUMP_CYCLE_PHASE:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, cycle_type, phase = parts
                print(f"MASTER: Water change cycle - {phase.replace('_', ' ')}")
                self.master_response_pub.publish(f"PUMP_RESPONSE: {phase.replace('_', ' ').title()}")
                
        elif response.startswith("PUMP_CYCLE_COMPLETED:"):
            parts = response.split(":", 2)
            if len(parts) >= 3:
                _, cycle_type, result = parts
                print(f"MASTER: Water change cycle completed - {result.replace('_', ' ')}")
                self.master_response_pub.publish(f"PUMP_RESPONSE: Water change completed - {result.replace('_', ' ')}")
                
    def water_level_callback(self, msg):
        """Handle water level sensor updates"""
        water_level_ok = msg.data
        
        if water_level_ok != self.water_level_ok:
            self.water_level_ok = water_level_ok
            status = "OK" if water_level_ok else "LOW"
            print(f"MASTER: Water level changed: {status}")

            if not water_level_ok:
                print("MASTER: LOW WATER LEVEL WARNING!")

    def publish_system_status(self, print_status=False):
        """Publish comprehensive system status"""
        system_status = {
            "bot_connected": self.bot_connected,
            "tank_status": self.tank_status,
            "tank_is_cleaning": self.tank_is_cleaning,
            "pump_running": self.pump_running,
            "pump_operation": self.pump_operation,
            "water_level_ok": self.water_level_ok,
            "last_command": self.last_user_command,
            "total_commands": len(self.command_history)
        }
        
        self.system_status_pub.publish(json.dumps(system_status))
        
        # Only print status when explicitly requested
        if print_status:
            print(f"MASTER: System status - Tank:{self.tank_status}, Pump:{self.pump_operation}, Water:{'OK' if self.water_level_ok else 'LOW'}")

    def monitor_system_health(self):
        """Monitor overall system health and coordination"""
        # Check for issues and handle coordination - only warn once per condition
        
        # Warning if tank is cleaning but water level is low
        if self.tank_is_cleaning and not self.water_level_ok:
            if not self.warned_low_water_cleaning:
                print("MASTER: WARNING - Tank cleaning with low water level!")
                self.warned_low_water_cleaning = True
        else:
            # Reset warning when condition is no longer true
            self.warned_low_water_cleaning = False

        # Warning if pump is running while tank is cleaning
        if self.tank_is_cleaning and self.pump_running:
            if not self.warned_pump_tank_simultaneous:
                print("MASTER: WARNING - Pump and tank cleaning running simultaneously!")
                self.warned_pump_tank_simultaneous = True
        else:
            # Reset warning when condition is no longer true
            self.warned_pump_tank_simultaneous = False

def main():
    try:
        master = MasterBrain()

        print("MASTER: System coordination active")
        print("MASTER: Ready to receive user commands via /master_control")
        print("MASTER: Monitoring all subsystem status")

        # Main coordination loop
        rate = rospy.Rate(1)  # 1 Hz system monitoring
        while not rospy.is_shutdown():
            master.publish_system_status()  # Don't print - just publish to ROS topics
            master.monitor_system_health()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        print("MASTER: ROS shutdown requested")
    except KeyboardInterrupt:
        print("\nMASTER: Keyboard interrupt received")
    except Exception as e:
        print(f"ERROR MASTER: Fatal error: {e}")
    finally:
        print("MASTER: Shutdown complete")

if __name__ == '__main__':
    main()
