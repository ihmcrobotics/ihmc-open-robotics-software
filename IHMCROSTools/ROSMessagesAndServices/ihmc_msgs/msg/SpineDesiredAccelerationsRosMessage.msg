## SpineDesiredAccelerationsRosMessage
# This message gives the user the option to bypass IHMC feedback controllers for the spine joints by
# sending desired joint accelerations. One needs experience in control when activating the bypass as
# it can result in unexpected behaviors for unreasonable accelerations. A message with a unique id
# equals to 0 will be interpreted as invalid and will not be processed by the controller.

# Specifies the desired joint accelerations.
float64[] desired_joint_accelerations

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


