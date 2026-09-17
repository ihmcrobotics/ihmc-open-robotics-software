# THIS MESSAGE IS SENT OVER VIDEO STREAMS.
# MAX SIZE OF THIS MESSAGE IS LIMITED (~881 bytes from a bit of testing).
# BE CAUTIOUS WHEN ADDING MORE FIELDS.
# If adding a field that only needs to be sent once,
# consider putting it in SRTStreamStatus.msg instead.

# Frame sequence number
uint32 sequence_number

# Frame acquisition time
ihmc_common_msgs/InstantMessage acquisition_time

# Sensor pose at time of acquisition
geometry_msgs/Pose sensor_pose
