# Message used to request a new LIDAR scan, and also enable some filters.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

bool remove_shadows true
bool remove_self_collisions true