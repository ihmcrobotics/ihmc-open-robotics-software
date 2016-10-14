## NeckDesiredAccelerationsRosMessage
# This message gives the user the option to bypass IHMC feedback controllers for the neck joints by
# sending desired neck joint accelerations. One needs experience in control when activating the bypass
# as it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id
# equals to 0 will be interpreted as invalid and will not be processed by the controller.

# Specifies the control mode for controlling the neck joints. See NeckControlMode.
uint8 neck_control_mode

# Specifies the desired joint accelerations. Only necessary when neckControlMode == USER_CONTROL_MODE.
float64[] neck_desired_joint_accelerations

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "neck_control_mode" enum values:
uint8 IHMC_CONTROL_MODE=0 # The IHMC controller controls the arm joints to execute desired inputs given from NeckTrajectoryMessage, HeadTrajectoryMessage. PD controllers are run for the given inputs and will either compute the desired head spatial acceleration or neck joint desired accelerations.The desired joint torques to achieve these desired accelerations are computed by the IHMC QP solver & inverse dynamics calculator.
uint8 USER_CONTROL_MODE=1 # The user directly sets what the neck joint desired accelerations have to be. The IHMC controller will stop tracking positions and the user desired accelerations will be fed to the IHMC QP solver & inverse dynamics to compute the desired joint torques.

