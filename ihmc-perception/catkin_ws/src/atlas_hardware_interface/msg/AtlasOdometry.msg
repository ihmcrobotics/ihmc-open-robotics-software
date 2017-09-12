Header header

sensor_msgs/Imu filtered_imu # Packet of filtered IMU data.

atlas_hardware_interface/AtlasPositionData pos_est # World position estimate for reference when giving desired step positions (not updated except in behavior modes where noted)
geometry_msgs/Vector3[2] foot_pos_est # World position estimate for feet (not updated except in behavior modes where noted)