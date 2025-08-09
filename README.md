# Tank Cleaning Robot ROS Package

A ROS-based autonomous tank cleaning robot that follows a systematic zig-zag pattern for thorough tank floor cleaning.

## Overview

This repository contains a complete ROS package for controlling an autonomous tank cleaning robot. The robot uses IR sensors for edge detection and follows a predefined zig-zag cleaning pattern to ensure complete coverage of the tank floor.

## Project Structure

```
tank_snode/
├── launch/
│   └── tank_bot.launch          # Launch file for all nodes
├── scripts/
│   ├── ir_sensor_node.py        # IR sensor interface
│   ├── motor_node.py            # Motor control
│   ├── decision_node.py         # Cleaning pattern logic
│   ├── comm_node.py             # WebSocket communication
│   ├── start_node.py            # System startup manager
│   └── README.md                # Detailed documentation
├── src/                         # C++ source files (if any)
├── CMakeLists.txt              # Build configuration
└── package.xml                 # Package metadata
```

## Features

- **Autonomous Navigation**: Systematic zig-zag cleaning pattern
- **Edge Detection**: IR sensor-based obstacle and edge avoidance
- **Remote Control**: WebSocket-based command interface
- **ROS Integration**: Full ROS node architecture with topic-based communication
- **Modular Design**: Separate nodes for sensors, motors, decision-making, and communication

## Getting Started

### Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Python 3.8+
- Catkin workspace

### Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone <your-github-repo-url>
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Install Python dependencies:
```bash
cd ~/catkin_ws/src
python3 -m venv tank_env
source tank_env/bin/activate
pip install -r requirements.txt  # if you create one
```

### Usage

1. **Launch the system**:
```bash
roslaunch tank_snode tank_bot.launch
```

2. **Start cleaning operation**:
Send a "start" command via WebSocket to begin the cleaning cycle.

## Cleaning Pattern

The robot follows this systematic zig-zag pattern:

```
START→ →→→→→→→→→→→→→→→→→→→→→→→→→→→→→2
                                    ↓
3←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←4
↓                                   ↓  
5→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→6
                                    ↓
7←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←8
↓                                   
9→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→END
```

## ROS Topics

- `/ir_sensor_data` - IR sensor readings
- `/motor_control` - Motor command messages
- `/robot_state` - Current robot status
- `/cleaning_command` - Start/stop cleaning commands

## Development

### Setting up Development Environment

1. Create and activate virtual environment:
```bash
python3 -m venv tank_env
source tank_env/bin/activate
```

2. Install development dependencies:
```bash
pip install rospy rospkg
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For questions or issues, please open an issue on GitHub or contact the maintainer.

---

**Built with care for autonomous robotics**
