# This message carries the general robot information such as the joints' state and IMU and force sensors' measurement.
# It is published frequently from the IHMC state estimator.

byte ROBOT_MOTION_STATUS_UNKNOWN=0
byte ROBOT_MOTION_STATUS_STANDING=1
byte ROBOT_MOTION_STATUS_IN_MOTION=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Time in nanoseconds of the clock hanging on the wall.
# Takes into account leap seconds/years and is updated by the NTP server (thus can jump backwards).
# The wall time is usually used in ROS1 for synchronizing timestamps of different time sources (computers, sensors, etc.)
int64 wall_time
# Time in nanoseconds that represents the absolute elapsed wall-clock time since some arbitrary, fixed point in the past.
# It is not affected by changes in the system time-of-day clock.
# This time is usually computed from a real-time process and can be used for reliably computing the time elapsed between two events.
int64 monotonic_time
# Platform dependent.
# Time signal in nanoseconds that can be used to synchronize two time sources.
int64 sync_timestamp
int32 joint_name_hash
float32[<=50] joint_angles
float32[<=50] joint_velocities
float32[<=50] joint_torques

geometry_msgs/Vector3 root_translation
geometry_msgs/Quaternion root_orientation
geometry_msgs/Vector3 pelvis_linear_velocity
geometry_msgs/Vector3 pelvis_angular_velocity
geometry_msgs/Vector3 pelvis_linear_acceleration

SpatialVectorMessage[<=50] force_sensor_data
IMUPacket[<=50] imu_sensor_data
byte robot_motion_status 255

int32 last_received_packet_type_id
int64 last_received_packet_unique_id
int64 last_received_packet_robot_timestamp