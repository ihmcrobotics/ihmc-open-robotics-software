atlas_hardware_interface/AtlasJointState[28] j # Current joint states
sensor_msgs/Imu filtered_imu # Packet of filtered IMU data.
atlas_hardware_interface/AtlasRawIMUData[15] raw_imu # Last n packets unfiltered IMU data.
geometry_msgs/Wrench[2] foot_sensors # State of foot sensors.
geometry_msgs/Wrench[2] wrist_sensors # State of wrist sensors.
atlas_hardware_interface/AtlasPositionData pos_est # World position estimate for reference when giving desired step positions (not updated except in behavior modes where noted)
atlas_hardware_interface/AtlasRobotBehavior current_behavior # Current behavior of robot.

