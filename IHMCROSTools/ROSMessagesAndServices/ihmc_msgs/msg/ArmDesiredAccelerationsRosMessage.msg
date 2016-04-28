## ArmDesiredAccelerationsRosMessage
# This message gives the user the option to bypass IHMC feedback controllers for the arm joints by
# sending desired arm joint accelerations. One needs experience in control when activating the bypass
# as it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id
# equals to 0 will be interpreted as invalid and will not be processed by the controller.

# Specifies the side of the robot that will execute the trajectory.
uint8 robot_side

# Specifies the control mode for controlling the arm joints. See ArmControlMode.
uint8 arm_control_mode

# Specifies the desired joint accelerations. Only necessary when armControlMode == USER_CONTROL_MODE.
float64[] arm_desired_joint_accelerations

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "arm_control_mode" enum values:
uint8 IHMC_CONTROL_MODE=0 # The IHMC controller controls the arm joints to execute desired inputs given from ArmTrajectoryMessage, HandTrajectoryMessage, and WholeBodyTrajectoryMessage. PD controllers are run for the given inputs and will either compute the desired hand spatial acceleration or arm joint desired accelerations.The desired joint torques to achieve these desired accelerations are computed by the IHMC QP solver & inverse dynamics calculator.
uint8 USER_CONTROL_MODE=1 # The user directly sets what the arm joint desired accelerations have to be. The IHMC controller will stop tracking positions and the user desired accelerations will be fed to the IHMC QP solver & inverse dynamics to compute the desired joint torques.

