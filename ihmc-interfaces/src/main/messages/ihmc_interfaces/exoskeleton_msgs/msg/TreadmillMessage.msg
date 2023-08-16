#This message is part of the Tufftread to Eva Exoskeleton Interface
#This message acts as a go-between between the Tufftread treadmill controller and Eva

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# bool to activate the treadmill
bool activate

# byte to decide current action
byte action 

# This is the data being sent to and from the treadmill
float64 speed
float64 incline

