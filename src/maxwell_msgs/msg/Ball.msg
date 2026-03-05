std_msgs/Header header
bool visible
# Global coordinates (map/world frame), meters
float32 global_x
float32 global_y
# Relative to robot (self frame), meters
float32 self_x
float32 self_y
# Pixel location (if available), -1 means unknown
int32 pixel_x
int32 pixel_y
# Camera angles (optional)
float32 alpha
float32 beta
