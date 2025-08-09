# Tank Cleaning Robot ROS System

A comprehensive ROS-based autonomous tank cleaning robot system with master-slave architecture and interactive testing capabilities.

## Overview

This repository contains a complete ROS system for controlling an autonomous tank cleaning robot. The system features a distributed architecture with separate master control and tank nodes, WebSocket communication, IR sensors for navigation, and an interactive testing interface.

## System Architecture

The system consists of two main packages:

- **tank_snode**: Autonomous tank robot with sensors, motors, and local intelligence
- **master_snode**: Master controller with WebSocket communication and testing interface

## Project Structure

```
├── master_snode/                    # Master control package
│   ├── launch/
│   │   └── master.launch           # Master node and test interface launcher
│   ├── scripts/
│   │   ├── master_node.py          # Master controller with WebSocket client
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
│   │   ├── motor_node.py           # Motor control
│   │   ├── start_node.py           # System startup manager
│   │   └── README.md               # Detailed technical documentation
│   ├── src/                        # C++ source files (if any)
│   ├── CMakeLists.txt
│   └── package.xml
├── myenv/                          # Python virtual environment
├── requirements.txt                # Python dependencies
└── README.md                       # This file
```

## Key Features

### Master-Slave Architecture
- **Master Node**: High-level control and user interface
- **Tank Node**: Autonomous robot with local intelligence
- **WebSocket Communication**: Real-time bidirectional communication
- **Distributed Processing**: Efficient load balancing

### Tank Robot Features
- **Autonomous Navigation**: Systematic zig-zag cleaning pattern
- **Edge Detection**: IR sensor-based obstacle and wall detection
- **Smart State Management**: Nodes only print on state changes
- **Quiet Initialization**: Professional startup with ready confirmation
- **Modular Design**: Separate nodes for sensors, motors, decision-making

### Master Control Features
- **WebSocket Client**: Connects to tank for command/status exchange
- **ROS Integration**: Publishes status and receives commands via topics
- **Status Publishing**: Real-time system status monitoring
- **Error Handling**: Robust connection management and reconnection

### Interactive Testing System
- **User-Friendly Interface**: Menu-driven command system
- **Multiple Input Methods**: Interactive, command-line, and automated modes
- **Status Reporting**: Formatted status displays showing bot state
- **Command Feedback**: Clear responses (Bot is running/Bot already running)
- **Professional Output**: Clean interface without spam or unnecessary output

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

### Usage

#### Option 1: Complete System Launch
```bash
# Terminal 1: Start tank nodes
roslaunch tank_snode tank_bot.launch

# Terminal 2: Start master controller with interactive interface  
roslaunch master_snode master.launch
```

#### Option 2: Manual Node Starting
```bash
# Start tank system first
roslaunch tank_snode tank_bot.launch

# In another terminal, start master node only
rosrun master_snode master_node.py

# In another terminal, start interactive test interface
rosrun master_snode test_user_input.py
```

#### Interactive Testing Commands
Once the system is running, use the test interface:

```
Available Commands:
  1. start   - Start tank cleaning operation
  2. stop    - Stop tank cleaning operation  
  3. status  - Get detailed tank status report
  4. help    - Show command menu
  5. quit    - Exit test program
```

## ROS Topics & Communication

### Tank Node Topics
- `/status_report` - Tank status messages and state updates
- `/tank_control` - Internal tank control signals (START_CLEANING/STOP_CLEANING)
- `/ir_sensor_data` - IR sensor readings and edge detection
- `/motor_commands` - Motor control commands and responses
- `/decision_state` - Decision node state and cleaning pattern info

### Master Node Topics  
- `/master_control` - Receives commands from test interface
- `/master_status` - Publishes system status and connection info
- `/master_commands` - Command confirmations and responses

### WebSocket Communication
- **Protocol**: JSON-based message exchange
- **Port**: 8765 (localhost)
- **Message Types**: 
  - `connection_established` - Initial handshake
  - `command_response` - Command execution results
  - `status_update` - Periodic tank status updates
  - `error_response` - Error handling and debugging

### Code Quality Standards

- **State-Change Logging**: Nodes only print when information changes  
- **Quiet Initialization**: Clean startup with ready confirmation messages
- **Modular Architecture**: Separate concerns across different nodes
- **Error Handling**: Comprehensive error management and recovery

### Architecture Notes

- **Master Node**: Acts as status publisher and command relay
- **Tank Node**: Autonomous operation with WebSocket server
- **Communication**: Asynchronous WebSocket with proper error handling
- **State Management**: Event-driven updates and clean status reporting

### Troubleshooting

**Connection Issues:**
- Ensure tank_snode is running before master_snode
- Check WebSocket port 8765 is available
- Verify virtual environment is activated

**Command Not Working:**
- Check ROS topics: `rostopic list`
- Monitor logs: `rosout` messages
- Use status command to verify system state

**Build Issues:**
- Clean build: `catkin_make clean && catkin_make`
- Check dependencies in requirements.txt
- Verify Python path and virtual environment

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Repository Information

- **Repository**: [ROS_FISHTANK_BOT](https://github.com/TheLostLeo/ROS_FISHTANK_BOT)
- **Author**: TheLostLeo
- **ROS Version**: Noetic (Ubuntu 20.04)
- **Python Version**: 3.8+


