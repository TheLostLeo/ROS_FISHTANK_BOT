#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import asyncio
import websockets
import threading

class CommNode:
    def __init__(self):
        self.status_pub = rospy.Publisher('/status_report', String, queue_size=10)
        self.control_pub = rospy.Publisher('/tank_control', String, queue_size=10)
        self.is_cleaning = False
        
    def start_cleaning(self):
        """Start the tank cleaning operation"""
        if not self.is_cleaning:
            self.is_cleaning = True
            rospy.loginfo("Received START signal - Beginning tank cleaning operation")
            self.status_pub.publish("Tank cleaning started")
            self.control_pub.publish("START_CLEANING")
        else:
            rospy.loginfo("Tank is already cleaning")
            
    def stop_cleaning(self):
        """Stop the tank cleaning operation"""
        if self.is_cleaning:
            self.is_cleaning = False
            rospy.loginfo("Received STOP signal - Stopping tank cleaning operation")
            self.status_pub.publish("Tank cleaning stopped")
            self.control_pub.publish("STOP_CLEANING")
        else:
            rospy.loginfo("Tank is not currently cleaning")

def comm_node():
    rospy.init_node('comm_node')
    comm = CommNode()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if comm.is_cleaning:
            comm.status_pub.publish("Tank is cleaning and active")
        else:
            comm.status_pub.publish("Tank is ready - waiting for start signal")
        rate.sleep()
        
    return comm

if __name__ == '__main__':
    async def websocket_server(websocket, path):
        comm = None
        try:
            async for message in websocket:
                rospy.loginfo(f"Received WebSocket message: {message}")
                
                if message.lower() == "start":
                    if comm:
                        comm.start_cleaning()
                    await websocket.send("Tank cleaning started")
                elif message.lower() == "stop":
                    if comm:
                        comm.stop_cleaning()
                    await websocket.send("Tank cleaning stopped")
                elif message.lower() == "status":
                    status = "cleaning" if comm and comm.is_cleaning else "ready"
                    await websocket.send(f"Tank status: {status}")
                else:
                    await websocket.send("Unknown command. Use: start, stop, or status")
                    
        except websockets.exceptions.ConnectionClosed:
            rospy.loginfo("WebSocket client disconnected")
        except Exception as e:
            rospy.logerr(f"WebSocket error: {e}")

    def run_websocket_server():
        """Run WebSocket server in a separate thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        start_server = websockets.serve(websocket_server, "localhost", 8765)
        rospy.loginfo("WebSocket server started on ws://localhost:8765")
        
        loop.run_until_complete(start_server)
        loop.run_forever()

    # Start WebSocket server in background thread
    websocket_thread = threading.Thread(target=run_websocket_server, daemon=True)
    websocket_thread.start()
    
    # Start the main communication node
    comm = comm_node()
    
    # Keep the main thread alive
    rospy.spin()
