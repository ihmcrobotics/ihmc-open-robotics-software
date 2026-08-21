# Generic hand configuration command. 0 means unused / do nothing.
# Other values are interpreted by the robot's hand controller.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Specifies the side of the robot of the hand being referred to
byte robot_side 255

# Hand configuration index. 0 means no command.
int32 configuration 0
