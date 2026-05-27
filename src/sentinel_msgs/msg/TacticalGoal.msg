std_msgs/Header header

uint8 NAVIGATE=0
uint8 CHASE_TARGET=1
uint8 RETREAT=2
uint8 HOLD=3
uint8 RUNE=4

uint8 goal_type
geometry_msgs/PoseStamped pose
float32 priority
float32 timeout_sec
string label
