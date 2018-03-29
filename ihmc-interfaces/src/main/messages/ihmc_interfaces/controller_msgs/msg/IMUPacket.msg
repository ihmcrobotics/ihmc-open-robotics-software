# General purpose packet to carry IMU data. (need to be replace with the ROS IMUData)

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
float64 time