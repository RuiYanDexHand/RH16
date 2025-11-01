# RH16 Hand Command Message - 16自由度手部命令消息
# 与C++版本完全对齐

# Header
std_msgs/Header header

# Hand identification
bool lr                    # left_or_right: false=left, true=right

# Control mode
int32 mode                 # 0: raw motor cmd, 1: joint angle cmd, 2: end position cmd

# Motor commands (16 motors)
int32[16] m_pos           # Motor target positions (0-4095)
int32[16] m_spd           # Motor speeds (0-4095)
int32[16] m_curlimit      # Motor current limits (0-4095)

# Joint angle commands (20 joints, 4 per finger)
float64[20] j_ang         # Target joint angles in radians

# Base coordinate system pose
float64 x_base            # Base X position
float64 y_base            # Base Y position
float64 z_base            # Base Z position
float64 roll_base         # Base roll angle
float64 pitch_base        # Base pitch angle
float64 yaw_base          # Base yaw angle

# Target fingertip positions (5 fingers) - for mode 2
float64[5] x              # Target fingertip X positions
float64[5] y              # Target fingertip Y positions
float64[5] z              # Target fingertip Z positions

# Target fingertip orientations (Euler angles) - for mode 2
float64[5] roll           # Target roll angles
float64[5] pitch          # Target pitch angles
float64[5] yaw            # Target yaw angles
