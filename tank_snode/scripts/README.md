# Tank Cleaning Robot System

## Overview
This system controls a tank cleaning robot that follows a systematic zig-zag cleaning pattern to ensure complete coverage of the tank floor.

## Cleaning Pattern
The robot follows this zig-zag pattern for thorough cleaning:

```
START→  1→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→2
                                                ↓
3←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←4
↓                                               ↓  
5→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→6
                                                ↓
7←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←8
↓                                               
9→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→END
```

- Starts at bottom-left (position 1)
- Moves right across the bottom
- When it hits the right edge, turns and moves up
- Then moves left across the tank
- Continues this pattern until the entire tank is cleaned

## Files Description

### Core Nodes
- `ir_sensor_node.py` - Reads IR sensor to detect edges/obstacles
- `motor_node.py` - Controls tank movement with enhanced turning capabilities
- `decision_node.py` - Implements zig-zag cleaning pattern and edge avoidance
- `comm_node.py` - Handles WebSocket communication for remote control
- `start_node.py` - Launches all nodes and manages the system

### Test Files
- `test_client.py` - Simple client to send commands via WebSocket

## How It Works

1. **System Startup**: Run `start_node.py` to start all components
2. **Standby Mode**: All nodes are active but motors are stopped, waiting for start command
3. **Start Cleaning**: Send "start" command via WebSocket to begin zig-zag cleaning operation
4. **Zig-Zag Cleaning Pattern**: 
   - Starts at bottom-left corner moving right
   - When hitting right edge, backs up, turns, and moves up slightly
   - Then moves left across the tank
   - When hitting left edge, backs up, turns, and moves up slightly  
   - Continues this back-and-forth pattern moving upward
   - Provides systematic coverage of the entire tank floor
5. **Edge Detection**: IR sensor detects edges and triggers pattern turns
6. **Stop Cleaning**: Send "stop" command to halt the cleaning operation

## Usage

### Start the System
```bash
cd /home/tll/catkin_ws/src/tank_snode/scripts
python3 start_node.py
```

### Send Commands
```bash
# Start cleaning
python3 test_client.py start

# Stop cleaning  
python3 test_client.py stop

# Check status
python3 test_client.py status
```

### WebSocket Commands
Connect to `ws://localhost:8765` and send:
- `start` - Begin tank cleaning operation
- `stop` - Stop tank cleaning operation  
- `status` - Get current tank status

## Dependencies
- ROS (Robot Operating System)
- RPi.GPIO (for Raspberry Pi GPIO control)
- websockets (for WebSocket communication)

Install websockets if needed:
```bash
pip3 install websockets
```

## ROS Topics
- `/edge_status` - IR sensor publishes edge detection status
- `/motor_command` - Commands sent to motor controller
- `/tank_control` - High-level tank control commands
- `/status_report` - System status messages
