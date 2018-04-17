# Atlas specific message

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

int32 VELOCITY = 0
int32 POSITION = 1

byte robot_side 255
float64 index
float64 middle
float64 thumb
float64 spread
int32 control_type