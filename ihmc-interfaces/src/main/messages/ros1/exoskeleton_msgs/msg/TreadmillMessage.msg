# This message is part of the Tufftread to Eva Exoskeleton Interface
# This message acts as a go-between between the Tufftread treadmill controller and Eva

int8 START_BELT_TIMEOUT_ENABLED=0

int8 START_BELT_TIMEOUT_DISABLED=1

int8 STOP_BELT=2

int8 SET_SPEED=3

int8 SET_ELEVATION=4

int8 AUTO_STOP=5

int8 AUTO_MINIMUM=6

int8 ACK_FLAG_TOGGLE=7

int8 BELT_STATUS=8

int8 GET_CURRENT_SPEED=9

int8 GET_CURRENT_ELEVATION=10

int8 GET_COMMANDED_SPEED=11

int8 GET_COMMANDED_ELEVATION=12

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# bool to dictate activation of communication between the treadmill and the computer. If true, activate and if false, deactivate
bool activate

# byte to decide current action based on the previously defined byte values
int8 action

# This is the data being sent to and from the treadmill. Speed is in MPH and incline is in percent incline
float64 speed

float64 incline


