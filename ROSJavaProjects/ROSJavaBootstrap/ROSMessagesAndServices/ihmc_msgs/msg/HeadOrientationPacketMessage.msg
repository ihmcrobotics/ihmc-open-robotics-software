## HeadOrientationPacketMessage
# This message gives the desired head orientation of the robot.
# On Atlas, head yaw will be achieved by moving the chest.

# quaternion gives the desired final orientation of the head
geometry_msgs/Quaternion quaternion
# desiredNeckPitechAngle specifies the desired pitch at the neck joint
float64 desiredNeckPitchAngle
# desiredJointForExtendedNeckPitchRange specifies the desired pitch at the joint used to extendthe head pitch range. On Atlas, this would be the waist.
float64 desiredJointForExtendedNeckPitchRangeAngle

