# This message is part of the IHMC whole-body controller API.
# This message gives the user the option to bypass IHMC feedback controllers for the arm joints by sending desired arm joint accelerations.
# One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations.
# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot that will execute the command.
byte robot_side 255
# The desired joint acceleration information.
DesiredAccelerationsMessage desired_accelerations