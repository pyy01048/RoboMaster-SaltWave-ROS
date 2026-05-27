std_msgs/Header header

# Target classes are intentionally generic so perception modules can evolve.
uint8 UNKNOWN=0
uint8 ARMOR=1
uint8 RUNE=2
uint8 BASE=3

uint8 target_type
uint8 target_id
float32 confidence

geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity

bool visible
bool shootable
float32 priority
