std_msgs/Header header

geometry_msgs/Pose2D self_pose
bool self_pose_sure

maxwell_msgs/Ball ball

# FallDirection: 0 none, +1 forward, -1 backward, +2 left, -2 right
int8 fall_direction

# support_foot: 0 double, 1 left, 2 right
uint8 support_foot
