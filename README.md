# Tank Cleaning Robot ROS System

A comprehensive ROS-based autonomous tank cleaning robot system with distributed master-brain architecture, water pump control, and professional communication interfaces.

## Overview

This repository contains a complete ROS system for controlling an autonomous tank cleaning robot with integrated water management. The system features a distributed architecture with centralized coordination, dedicated subsystem controllers, WebSocket communication, and professional status reporting.

## System Architecture

The system consists of two main packages with a centralized brain architecture:

- **tank_snode**: Autonomous tank robot with sensors, motors, and local intelligence
- **master_snode**: Central coordination brain with specialized subsystem controllers

### Master-Brain Architecture

```
User Interface → Master Brain → Subsystem Controllers → Hardware/Tank
     ↑              ↓              ↓                    ↓
Response Topics ← Status Updates ← Component Status ← Actual Operations
```

## Project Structure

```
├── master_snode/                    # Master coordination package
│   ├── launch/
│   │   └── master.launch           # Complete system launcher
│   ├── scripts/
│   │   ├── master_node.py          # Central brain coordinator  
│   │   ├── bot_controller.py       # WebSocket communication handler
│   │   ├── water_pump_controller.py # Demo water pump control
│   │   └── test_user_input.py      # Interactive testing interface
│   ├── CMakeLists.txt
│   └── package.xml
├── tank_snode/                      # Tank robot package  
│   ├── launch/
│   │   └── tank_bot.launch         # Launch file for all tank nodes
│   ├── scripts/
│   │   ├── comm_node.py            # WebSocket server & communication
│   │   ├── decision_node.py        # Cleaning pattern logic
│   │   ├── ir_sensor_node.py       # IR sensor interface
│   │   └── motor_node.py           # Motor control (demo)

│   ├── urdf/                       # Robot models for simulation
│   ├── worlds/                     # Gazebo world files
│   ├── src/                        # C++ source files (if any)
│   ├── CMakeLists.txt
│   └── package.xml
├── .env                            # Environment variables (user-created)
├── example.env                     # Environment variable template
├── .gitignore                      # Version control exclusions
└── README.md                       # This file
```

## Package Dependencies

### Required ROS Packages
- **rospy** - Python client library for ROS
- **std_msgs** - Standard ROS message types
- **sensor_msgs** - Sensor-specific message types
- **geometry_msgs** - Geometric primitive messages
- **catkin** - ROS build system

## Getting Started

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Python 3.8+
- Catkin workspace

### Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/TheLostLeo/ROS_FISHTANK_BOT.git
```

2. Build the packages:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Set up Python virtual environment:
```bash
cd ~/catkin_ws/src
python3 -m venv myenv
source myenv/bin/activate
pip install -r requirements.txt  
```

3. Configure environment variables:
```bash
# Copy the example environment file
cp example.env .env

# Edit .env file to customize WebSocket settings (optional)
# Default: WEBSOCKET_HOST=localhost, WEBSOCKET_PORT=8765
nano .env  # or use your preferred editor
```

### Usage

### Usage

#### Complete System Launch (Recommended)
```bash
# Terminal 1: Start tank robot system
roslaunch tank_snode tank_bot.launch

# Terminal 2: Start master brain with all controllers
roslaunch master_snode master.launch
```

#### Interactive Testing Commands
The test interface provides comprehensive control over the entire system:

```
Available Commands:
TANK CONTROLS:
  1. start         - Start tank cleaning operation
  2. stop          - Stop tank cleaning operation 
  3. status        - Get current tank status
PUMP CONTROLS:
  4. drain         - Drain tank water
  5. fill          - Fill tank with fresh water
  6. water_change  - Complete water change cycle
  7. pump_stop     - Stop all pump operations
SYSTEM:
  8. system_status - Get complete system status
  9. help          - Show this menu
 10. quit          - Exit test program
```
## Key Features

### Central Brain Architecture
- **Master Brain**: Central coordinator managing all subsystems via ROS topics
- **Bot Controller**: Dedicated WebSocket handler for tank communication
- **Water Pump Controller**: Hardware control with simulation fallback
- **Professional Communication**: Status updates only when changes occur

### Advanced System Coordination
- **Topic-Based Communication**: Pure ROS messaging between all components
- **State-Change Reporting**: Smart status updates prevent message spam
- **One-Time Warnings**: Health monitoring without repetitive alerts
- **Comprehensive Status**: Complete system status with JSON formatting

### Tank Robot Features
- **Autonomous Navigation**: Systematic zig-zag cleaning pattern
- **Edge Detection**: IR sensor-based obstacle and wall detection
- **Smart State Management**: Nodes only print on state changes
- **Quiet Initialization**: Professional startup with ready confirmation
- **Modular Design**: Separate nodes for sensors, motors, decision-making

### Water Management System
- **Demo Pump Control**: Simulated water pump operations without hardware
- **Operation Feedback**: Start/stop/completion messages to master brain
- **Water Level Simulation**: Realistic water level changes during operations
- **Multi-Phase Cycles**: Complete water change cycles (drain → fill)
- **Safety Monitoring**: Warnings for simultaneous pump/cleaning operations

### Interactive Testing System
- **Enhanced Command Menu**: Tank controls + pump controls + system status
- **Professional Output**: Clean status reports without spam
- **Multiple Operation Modes**: Interactive, command-line, and automated testing
- **Real-Time Feedback**: Immediate command acknowledgment and status updates
- **Comprehensive Controls**: 
  - Tank: `start`, `stop`, `status`
  - Pump: `drain`, `fill`, `water_change`, `pump_stop`
  - System: `system_status`

### Configuration Management
- **Environment Variables**: Secure WebSocket configuration via .env files
- **python-dotenv Integration**: Professional environment variable handling
- **Security**: Sensitive configuration excluded from version control (.gitignore)
- **Easy Setup**: Copy example.env to .env for quick configuration
- **Default Values**: Works out-of-the-box with localhost:8765


## ROS Topics & Communication

### Master Brain Topics
- `/master_control` - Receives commands from user interface
- `/master_status` - Publishes comprehensive system status (JSON)
- `/master_commands` - Command confirmations and responses
- `/bot_commands` - Sends commands to tank via bot controller
- `/pump_control` - Sends commands to water pump controller

### Tank Communication Topics
- `/tank_response` - Tank command responses via bot controller
- `/tank_status` - Tank status updates (cleaning/idle states)
- `/bot_connection` - Bot controller connection status

### Water Pump Topics
- `/pump_status` - Pump operation status (running, operation, flow rate)
- `/pump_response` - Pump operation feedback (start/stop/completion)
- `/water_level` - Water level sensor data (OK/LOW)
- `/flow_rate` - Current pump flow rate information

### Tank Node Internal Topics  
- `/tank_control` - Internal tank control signals
- `/ir_sensor_data` - IR sensor readings and edge detection
- `/motor_commands` - Motor control commands and responses
- `/decision_state` - Decision node state and cleaning patterns

### WebSocket Communication
- **Protocol**: JSON-based message exchange between bot controller and tank
- **Default Port**: 8765 (configurable via .env)
- **Message Types**: 
  - Command execution (`start`, `stop`, `status`)
  - Status updates (cleaning state, system health)
  - Error handling and connection management
- **Security**: Host/port configured via environment variables

### Code Quality Standards

- **Professional Output**: Nodes print only on state changes, not continuous status
- **One-Time Warnings**: System health warnings appear once per condition
- **State-Change Logging**: Smart status updates prevent console spam  
- **Quiet Initialization**: Clean startup with ready confirmation messages
- **Modular Architecture**: Clear separation of concerns across specialized nodes
- **Comprehensive Error Handling**: Robust error management and recovery
- **Topic-Based Coordination**: Pure ROS communication between all components

### Architecture Highlights

- **Central Brain Pattern**: Master node coordinates all subsystem controllers
- **Specialized Controllers**: Dedicated nodes for WebSocket, pump, and coordination
- **Professional Communication**: Status updates only when information changes
- **Smart Status Management**: Prevents repetitive messages and console spam
- **Real-Time Feedback**: Immediate acknowledgment of commands with operation progress
- **Comprehensive Monitoring**: System health warnings with intelligent state tracking

### Troubleshooting

**System Startup:**
- Start tank_snode first, then master_snode
- Ensure .env file exists: `cp example.env .env`
- Verify all nodes launch successfully in separate terminals

**Connection Issues:**
- Check WebSocket port availability (default: 8765)
- Verify bot controller connects to tank communication node
- Monitor connection status via `/bot_connection` topic

**Command Not Working:**
- Use `system_status` command to verify system state
- Check ROS topics: `rostopic list` and `rostopic echo /master_commands`
- Monitor master brain coordination messages
- Verify all subsystem controllers are running

**Water Pump Issues:**
- Pump controller runs in demo mode (no hardware required)
- Check `/pump_response` topic for operation feedback
- Monitor `/pump_status` for current operation state
- Use `pump_stop` to halt any running operations

**Environment Variable Issues:**
- Missing .env file: Copy `example.env` to `.env`
- Port conflicts: Change `WEBSOCKET_PORT` in .env file
- Host binding issues: Set `WEBSOCKET_HOST=0.0.0.0` for network access

**Build Issues:**
- Clean build: `catkin_make clean && catkin_make`
- Check all required ROS packages are installed
- Verify Python dependencies
- Ensure proper workspace sourcing: `source devel/setup.bash`

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Repository Information

- **Repository**: [ROS_FISHTANK_BOT](https://github.com/TheLostLeo/ROS_FISHTANK_BOT)
- **Author**: TheLostLeo
- **ROS Version**: Noetic (Ubuntu 20.04)
- **Python Version**: 3.8+


