


<<<<<<< HEAD
# This message is part of the Quix controller API.

# This message is used to notify the crutch display of the current motion state and allow the crutch to communicate a desired change in state.














=======
# This message is part of the IHMC whole-body controller API.

# This message is used to switch the control scheme between different control mode.
>>>>>>> develop















int8 HOLD_POSITION=0






int8 SIT_DOWN=1






int8 STAND_UP=2






<<<<<<< HEAD
int8 MOVE_TO_FLAT_GROUND=3





=======
int8 MOVE_TO_START=3
>>>>>>> develop






int8 FLAT_GROUND_WALKING=4






<<<<<<< HEAD
int8 MOVE_TO_SLOPE=5






int8 SLOPE_WALKING=6






int8 OLD_FLAT_GROUND_WALKING=7






=======
>>>>>>> develop

# Unique ID used to identify this message, should preferably be consecutively increasing.



uint32 sequence_id





<<<<<<< HEAD
=======
# Specifies the which state the controller should transition into.



>>>>>>> develop
# Field default value 255

int8 motion_state_name



