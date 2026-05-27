std_msgs/Header header

# System status
uint8 OK=0
uint8 WARNING=1
uint8 ERROR=2
uint8 CRITICAL=3

uint8 status

# Module health
bool imu_healthy
bool lidar_healthy
bool vision_healthy
bool graph_opt_healthy

# Timing info
float32 imu_rate
float32 lidar_rate
float32 vision_rate
float32 graph_rate

# Quality metrics
float32 position_confidence
float32 orientation_confidence
