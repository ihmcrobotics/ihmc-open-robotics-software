# Atlas specific message.

int8 NONE=0

int8 FREEZE=1

int8 STAND_PREP=2

int8 STAND=3

int8 WALK=4

int8 STEP=5

int8 MANIPULATE=6

int8 USER=7

int8 CALIBRATE=8

int8 SOFT_STOP=9

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 atlas_bdi_robot_behavior

bool stop


