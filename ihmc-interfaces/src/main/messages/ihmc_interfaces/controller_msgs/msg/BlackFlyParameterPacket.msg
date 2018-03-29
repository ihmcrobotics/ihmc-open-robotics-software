# Atlas specific message

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

bool auto_exposure
bool auto_gain
bool auto_shutter
float64 exposure
float64 frame_rate
bool from_ui
float64 gain
float64 shutter
byte robot_side 255