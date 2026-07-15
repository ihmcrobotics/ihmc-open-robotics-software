# This message is used to start and stop recording ROS logs.
# This is used both for recording and replaying, see ROS2LogRecord and ROS2LogReplay for more information.

byte ROS2_LOG_REQUESTED_STATE_START=0
byte ROS2_LOG_REQUESTED_STATE_FINISH=1

# Requested state for the ROS 2 log recorder
byte requested_state 255