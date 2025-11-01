# RH16 ROS2 Python — Commands Quick Guide

## Build/Install
```bash
cd ryhand_sdk/py
./build_all.sh   # or ./rebuild.sh
```

## Single hand run
- Left on can0 (example):
```bash
ros2 run rh16_ctrl_py rh16_ctrl_node --ros-args -p hand_type:=left -p can_interface:=can0
# Test node
ros2 run rh16_ctrl_py rh16_test_cpp_aligned --ros-args -p mode:=0 -p lr:=0
```

## Dual hands run (one command)
Use provided launch files:
```bash
# Auto/Detect or manual interfaces (check available files):
ros2 launch rh16_ctrl_py dual_hand_control.launch.py
# Or manual CAN selection:
ros2 launch rh16_ctrl_py dual_hand_manual_can.launch.py can_left:=can0 can_right:=can1
```

Notes:
- Ensure CAN up: `sudo ip link set canX up type can bitrate 1000000`.
- Review launch/README in `rh16_ctrl_py/launch` for more options.