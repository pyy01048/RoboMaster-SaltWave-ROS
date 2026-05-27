std_msgs/Header header

# Tracking ID (persistent across frames)
uint32 track_id

# Bounding box
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Class info
uint8 class_id
string class_name
float32 confidence

# 3D info
geometry_msgs/Pose pose
float32 depth

# Velocity estimate (from Kalman filter)
geometry_msgs/Vector3 velocity

# Tracking info
uint32 age
uint32 hit_streak
float32 time_since_update
