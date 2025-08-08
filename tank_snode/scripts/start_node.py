#!/usr/bin/env python3
import rospy
import subprocess
import os
import sys
import signal
import time

class BotController:
    def __init__(self):
        self.processes = []
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
    def start_node(self, node_script):
        """Start a ROS node script"""
        script_path = os.path.join(self.script_dir, node_script)
        try:
            process = subprocess.Popen([sys.executable, script_path])
            self.processes.append(process)
            rospy.loginfo(f"Started {node_script} with PID: {process.pid}")
            time.sleep(1)  # Give each node time to initialize
            return process
        except Exception as e:
            rospy.logerr(f"Failed to start {node_script}: {e}")
            return None
    
    def stop_all_nodes(self):
        """Stop all running nodes"""
        rospy.loginfo("Stopping all nodes...")
        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
            except Exception as e:
                rospy.logwarn(f"Error stopping process: {e}")
        self.processes.clear()
    
    def start_bot(self):
        """Start all bot nodes in the correct order"""
        rospy.init_node('start_node')
        rospy.loginfo("Starting Tank Bot System...")
        
        # Start nodes in order
        nodes_to_start = [
            'ir_sensor_node.py',
            'motor_node.py', 
            'decision_node.py',
            'comm_node.py'
        ]
        
        for node in nodes_to_start:
            if self.start_node(node):
                rospy.loginfo(f"Successfully started {node}")
            else:
                rospy.logerr(f"Failed to start {node}")
                self.stop_all_nodes()
                return False
        
        rospy.loginfo("All nodes started successfully!")
        return True
    
    def monitor_nodes(self):
        """Monitor running nodes and restart if needed"""
        rate = rospy.Rate(1)  # Check every second
        
        while not rospy.is_shutdown():
            # Check if any processes have died
            for i, process in enumerate(self.processes):
                if process.poll() is not None:  # Process has terminated
                    rospy.logwarn(f"Node {i} has died, attempting restart...")
                    # Could implement restart logic here if needed
            
            rate.sleep()

def signal_handler(signum, frame):
    """Handle shutdown signals"""
    rospy.loginfo("Received shutdown signal")
    bot_controller.stop_all_nodes()
    sys.exit(0)

if __name__ == '__main__':
    bot_controller = BotController()
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        if bot_controller.start_bot():
            rospy.loginfo("Bot system is running. Press Ctrl+C to stop.")
            bot_controller.monitor_nodes()
        else:
            rospy.logerr("Failed to start bot system")
            sys.exit(1)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown requested")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        bot_controller.stop_all_nodes()
        rospy.loginfo("Bot system stopped")
