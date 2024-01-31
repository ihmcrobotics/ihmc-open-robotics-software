#This message is part of the Tufftread to Eva Exoskeleton Interface
#This message acts as a go-between between the Tufftread treadmill controller and Eva

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte START_BELT_TIMEOUT_ENABLED = 0
byte START_BELT_TIMEOUT_DISABLED = 1
byte STOP_BELT = 2
byte SET_SPEED = 3
byte SET_ELEVATION = 4
byte AUTO_STOP = 5
byte AUTO_MINIMUM = 6
byte ACK_FLAG_TOGGLE = 7
byte BELT_STATUS = 8
byte GET_CURRENT_SPEED = 9
byte GET_CURRENT_ELEVATION = 10
byte GET_COMMANDED_SPEED = 11
byte GET_COMMANDED_ELEVATION = 12

# bool to dictate activation of communication between the treadmill and the computer. If true, activate and if false, deactivate
bool activate

# byte to decide current action based on the previously defined byte values
byte action 

# This is the data being sent to and from the treadmill. Speed is in MPH and incline is in percent incline
float64 speed
float64 incline

