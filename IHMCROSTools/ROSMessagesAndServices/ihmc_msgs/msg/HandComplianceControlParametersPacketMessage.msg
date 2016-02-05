## HandComplianceControlParametersPacketMessage
# This message setup the hand controller to activate a compliance module if at least one of the fields is set and deactivate if the message is empty.
# To compliance module works only when the last hand command sent with the HandPosePacket is a hand pose (not joint angles).
# Once activated, an integrator is used on the error in force/torque measured to keep adjusting the desired hand pose until the desired
# force/torque are achieved or until the maximum correction is reached (set to 5cm for translation and 0.2rad for the orientation).
# As it uses the measurements from wrist force sensors, a calibration of these is preferred prior to activation of compliance.

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

# enableLinearCompliance allows to activate/deactivate the compliance in translation for each individual axes (X, Y, and Z).
# The axes are in the hand control frame attached to the hand:
#  - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),
#  - Y refers to the grasping axis (e.g. left/right),
#  - Z refers to the axis orthogonal to the two other axes (e.g. up/down).
# If the field is null, the linear compliance will be deactivated.
bool[] enable_linear_compliance

# enableAngularCompliance allows to activate/deactivate the compliance in orientation for each individual axes (X, Y, and Z).
# The axes are in the hand control frame attached to the hand:
#  - X refers to the axis perpendicular to the hand palm,
#  - Y refers to the grasping axis,
#  - Z refers to the axis orthogonal to the two other axes.
# If the field is null, the angular compliance will be deactivated.
bool[] enable_angular_compliance

# desiredForce allows to set the desired force to be achieved on the hand for each individual axes (X, Y, and Z).
# The axes are in the hand control frame attached to the hand:
#  - X refers to the axis perpendicular to the hand palm (e.g. forward/backward),
#  - Y refers to the grasping axis (e.g. left/right),
#  - Z refers to the axis orthogonal to the two other axes (e.g. up/down).
# If the field is null, the desired force will be set to zero.
geometry_msgs/Vector3 desired_force

# desiredTorque allows to set the desired torque to be achieved on the hand for each individual axes (X, Y, and Z).
# The axes are in the hand control frame attached to the hand:
#  - X refers to the axis perpendicular to the hand palm,
#  - Y refers to the grasping axis,
#  - Z refers to the axis orthogonal to the two other axes.
# If the field is null, the desired torque will be set to zero.
geometry_msgs/Vector3 desired_torque

# wrenchDeadzones set the deadzones that are used on the force and torque measurements, respectively.
# For instance, if wrenchDeadzones = {5.0, 0.5}, the controller will perceive only forces that are outside the range [-5N, 5N],
# and torques that are outside the range [-0.5N.m, 0.5N.m].
# As results, the compliance control will start adjusting the desired hand pose only for measured forces/torques greater
# than the specified deadzones.
# If the field is null, the deadzone won't be changed.
# We have found that wrenchDeadzones = {10.0, 0.5} does not affect much the position control accuracy of the hand but still gives good compliance.
float32[] wrench_deadzones

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


