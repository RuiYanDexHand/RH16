# RH16 ROS2 C++ — Commands Quick Guide

## Build (ROS2)
1. Source ROS2 env
2. Build with colcon:
```bash
cd ryhand_sdk/cpp
./ros2build.sh
```

## Single hand run
- Left hand on can0, right hand on can1 (adjust as needed)
```bash
# Controller (example executable names; replace if different in your workspace)
ros2 run rh16_ctrl rh16_ctrl_node --ros-args -p hand_type:=left -p can_interface:=can0

# Sine test publisher
ros2 run rh16_ctrl rh16_test --ros-args -p mode:=0 -p lr:=0
```

## Dual hands run (one command)
If your package provides a dual-hand launch (BothHands.launch.py exists):
```bash
ros2 launch rh16_ctrl BothHands.launch.py can_left:=can0 can_right:=can1
```
Otherwise run two controllers and a single test publisher:
```bash
# Term 1
ros2 run rh16_ctrl rh16_ctrl_node --ros-args -p hand_type:=left  -p can_interface:=can0
# Term 2
ros2 run rh16_ctrl rh16_ctrl_node --ros-args -p hand_type:=right -p can_interface:=can1
# Term 3
ros2 run rh16_ctrl rh16_test --ros-args -p mode:=0 -p lr:=0
```

Notes:
- Ensure the CAN interfaces are up: `sudo ip link set canX up type can bitrate 1000000`.
- Topic names may differ; align with your controller/test defaults.