## HandPosePacketMessage
# This message commands the controller to move an arm end effector to a given
# position and orientation. On Atlas, the end effector position is considered
# as the end of the hand attachment plate, which is about 10 cm from the end
# of the wrist. The position/orientation may be specified in world frame, chest
# frame, or desired joint angles.

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

# data_type specifies whether or not the IHMC Controller should use the pose fields
# or the joint angles array for computing control output for the arms
# Options for dataType
uint8 HAND_POSE=0 # the hand pose will be represented by a hand position and a hand orientation (joint angles will be ignored)
uint8 JOINT_ANGLES=1 # the joint angles contained in this package will be used for control (position and orientation will be ignored)
uint8 data_type

# when using HAND_POSE datatype commands, specify whether the pose should be held in world or chest frame. Note that regardless of the frame specified here the position and orientation must be expressed in world frame.
# Options for referenceFrame
uint8 CHEST=0 # frame attached to the chest of the robot
uint8 WORLD=1 # world frame
uint8 reference_frame

# to_home_position can be used to move the arm end effectors back to their starting
# position, defined as down and beside the robot with slightly bent elbows
bool to_home_position

# the position component of a HAND_POSE type message. See the data_type field.
geometry_msgs/Vector3 position

# the orientation component of a HAND_POSE type message. See the data_type field.
geometry_msgs/Quaternion orientation

# trajectory_time specifies how fast or how slow to move to the desired pose
float64 trajectory_time

# joint_angles specifies the desired arm joint angles in order for a JOINT_ANGLES type messages.For Atlas V5 the controller assumes joint angles will be given in the following order:
# shoulder yaw, shoulder roll, elbow pitch, elbow roll, upper wrist pitch, wrist roll, lower wrist pitch
float64[] joint_angles

# Specifies whether or not the orientation of the hand should be controller during HAND_POSE commands.
bool control_orientation

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


