std_msgs/Header header

# Bounding box
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Class info
uint8 class_id
string class_name
float32 confidence

# 3D info (if available)
geometry_msgs/Pose pose
float32 depth

# Velocity estimate
geometry_msgs/Vector3 velocity
