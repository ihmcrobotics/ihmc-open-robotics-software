# Atlas specific message

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool auto_exposure
bool auto_gain
bool auto_shutter
float64 exposure
float64 frame_rate
bool from_ui
float64 gain
float64 shutter
byte robot_side 255