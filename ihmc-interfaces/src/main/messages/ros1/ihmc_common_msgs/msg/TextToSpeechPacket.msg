# Use this message for the robot to speak.

string FOOTSTEP_COMPLETED="Finished Taking Footstep"

string FINISHED_WALKING="Finished walking"

string WALKING_ABORTED="walking aborted"

string STARTING_CONTROLLER="starting controller"

string WALKING="walking"

string MOVING_LEFT_ARM="moving the left arm"

string MOVING_RIGHT_ARM="moving the right arm"

string NETWORKPROCESSOR_ONLINE="Reestablished Connection To The Network Processor"

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

bool speak_packet

# Field default value True
bool beep

string text_to_speak


