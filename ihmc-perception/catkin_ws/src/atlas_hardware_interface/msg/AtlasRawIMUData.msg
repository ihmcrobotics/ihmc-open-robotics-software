# Structure for returned raw IMU readings.
time imu_timestamp # Timestamp for IMU data [s*(10^-6)].
uint64 packet_count # Packet number from IMU
geometry_msgs/Vector3 da # Delta angle (radians) in the frame of the IMU.
geometry_msgs/Vector3 dd # Linear acceleration (m/s^2) in the frame of the IMU. The location of the sensor differs in each direction.
