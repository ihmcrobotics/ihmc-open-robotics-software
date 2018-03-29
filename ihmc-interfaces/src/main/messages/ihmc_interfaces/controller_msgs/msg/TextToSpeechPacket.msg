# Use this message for the robot to speak.

string FOOTSTEP_COMPLETED = "Finished Taking Footstep"
string FINISHED_WALKING = "Finished walking"
string WALKING_ABORTED = "walking aborted"
string STARTING_CONTROLLER = "starting controller"
string WALKING = "walking"
string MOVING_LEFT_ARM = "moving the left arm"
string MOVING_RIGHT_ARM = "moving the right arm"
string NETWORKPROCESSOR_ONLINE = "Reestablished Connection To The Network Processor"

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

bool speak_packet false
bool beep true
string text_to_speak