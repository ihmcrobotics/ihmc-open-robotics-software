# This message is part of the IHMC whole-body controller API.
# It is a lightweight version of WholeBodyTrajectoryMessage designed for streaming.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When receiving a trajectory message that is part of a stream, the controller will extrapolate the trajectory point in the future using a simple first order integration over the given duration.
# This integration allows to improve continuity of execution for streams.
# If no new message is received once the integration duration has elapsed, the controller will hold the desired position and reset the desired velocity to 0.
float32 stream_integration_duration

# Timestamp (in nanoseconds) when this message was created.
# The timestamp can be generated from the computer where this message originates.
int64 timestamp

# Information for the left hand in task-space
bool has_left_hand_streaming_message

ihmc_common_msgs/SE3StreamingMessage left_hand_streaming_message

# Information for the right hand in task-space
bool has_right_hand_streaming_message

ihmc_common_msgs/SE3StreamingMessage right_hand_streaming_message

# Information for the left arm joints
bool has_left_arm_streaming_message

controller_msgs/JointspaceStreamingMessage left_arm_streaming_message

# Information for the right arm joints
bool has_right_arm_streaming_message

controller_msgs/JointspaceStreamingMessage right_arm_streaming_message

# Information for the chest
bool has_chest_streaming_message

ihmc_common_msgs/SO3StreamingMessage chest_streaming_message

# Information for the pelvis
bool has_pelvis_streaming_message

# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
bool enable_user_pelvis_control

ihmc_common_msgs/SE3StreamingMessage pelvis_streaming_message

# Information for the neck joints
bool has_neck_streaming_message

controller_msgs/JointspaceStreamingMessage neck_streaming_message


