# Header (time and frame)
Header header

# The step number of the foothold.
# Multiple alternative footholds per step number are possible.
uint8 stepNumber

# Foot type identifier (left, right etc.)
std_msgs/String type

# Footstep position and orientation
geometry_msgs/Pose pose

# Status flag (0: unknown, 1: verified, 2: do not change)
int8 flag
