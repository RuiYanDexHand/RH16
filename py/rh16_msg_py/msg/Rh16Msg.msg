# RH16 Hand Status Message - 16自由度手部状态消息
# 与C++版本完全对齐

# Header
std_msgs/Header header

# Hand identification
bool lr                    # left_or_right: false=left, true=right

# Motor status (16 motors)
int32[16] status          # Motor status
int32[16] m_pos           # Motor positions (0-4095)
int32[16] m_spd           # Motor speeds
int32[16] m_cur           # Motor currents
int32[16] m_force         # Motor forces

# Joint angles (20 joints, 4 per finger)
float64[20] j_ang         # Joint angles in radians

# Base coordinate system pose
float64 x_base            # Base X position
float64 y_base            # Base Y position  
float64 z_base            # Base Z position
float64 roll_base         # Base roll angle
float64 pitch_base        # Base pitch angle
float64 yaw_base          # Base yaw angle

# Fingertip positions (5 fingers)
float64[5] x              # Fingertip X positions
float64[5] y              # Fingertip Y positions
float64[5] z              # Fingertip Z positions

# Fingertip orientations (quaternions)
float64[5] w              # Quaternion w components
float64[5] i              # Quaternion x components  
float64[5] j              # Quaternion y components
float64[5] k              # Quaternion z components

# Fingertip orientations (Euler angles)
float64[5] roll           # Roll angles
float64[5] pitch          # Pitch angles
float64[5] yaw            # Yaw angles
