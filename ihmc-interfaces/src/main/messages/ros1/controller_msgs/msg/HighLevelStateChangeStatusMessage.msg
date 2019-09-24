int8 DO_NOTHING_BEHAVIOR=0

int8 STAND_PREP_STATE=1

int8 STAND_READY=2

int8 FREEZE_STATE=3

int8 STAND_TRANSITION_STATE=4

int8 WALKING=5

int8 EXIT_WALKING=6

int8 DIAGNOSTICS=7

int8 CALIBRATION=8

int8 CUSTOM1=9

int8 FALLING_STATE=10

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the controller's state prior to transition.
# Field default value 255
int8 initial_high_level_controller_name

# Specifies the state the controller has transitioned into.
# Field default value 255
int8 end_high_level_controller_name


