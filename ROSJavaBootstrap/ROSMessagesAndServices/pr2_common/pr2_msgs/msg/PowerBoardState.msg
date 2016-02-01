# This message communicates the state of the PR2's power board.
int8 STATE_NOPOWER=0
int8 STATE_STANDBY=1
int8 STATE_PUMPING=2
int8 STATE_ON=3
int8 STATE_ENABLED=3  # Preferred over STATE_ON, keeping STATE_ON for backcompat
int8 STATE_DISABLED=4

int8 MASTER_NOPOWER=0
int8 MASTER_STANDBY=1
int8 MASTER_ON=2
int8 MASTER_OFF=3
int8 MASTER_SHUTDOWN=4

Header header
string name # Name with serial number
uint32 serial_num # Serial number for this board's message
float64 input_voltage # Input voltage to power board

# Master States:
#  MASTER_NOPOWER, MASTER_STANDBY, MASTER_ON, MASTER_OFF, MASTER_SHUTDOWN 
int8 master_state  # The master state machine's state in the powerboard

# Circuit States:
#  STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED
int8[3] circuit_state # One of the above states
float64[3] circuit_voltage  # Output voltage of each circuit

# True if robot should be enabled
bool run_stop           #Note - if the wireless run-stop is hit, this will be unobservable
bool wireless_stop 
