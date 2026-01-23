# dan_ros_motor

## Overview
This package is designed for motor control in ROS 2 systems. It can be applied to robots that require speed or direction control of motors via ROS 2 topics.

## Related Structure
- `scripts/` : Python scripts for various nodes
- `launch/` : Launch files for running nodes
- `config/` : Parameter or other config files
- `src/` : C++ code (if any)
- `description/` : URDF or model files

## Usage Examples

### Running a launch file (example)
```bash
ros2 launch dan_ros_motor dan_motor.launch.py
```

### Running a node directly (example)
```bash
ros2 run dan_ros_motor motor_danail_pdo
```

## Customization
- You can add/modify scripts in `scripts/` and create new launch files in `launch/`.
- Parameters can be adjusted in the config files.

## Recommendations
- Make sure all required dependencies are listed in `package.xml` and `CMakeLists.txt`.
- If you want to add a new node, it is recommended to create a new file instead of modifying existing ones.