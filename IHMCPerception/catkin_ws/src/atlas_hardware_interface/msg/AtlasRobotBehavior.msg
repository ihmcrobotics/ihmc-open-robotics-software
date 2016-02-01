# Enum representing ATLAS's behavior
uint8 state

uint8 BEHAVIOR_NONE = 0 # Robot currently not in any behavior; software freeze, controls disabled.
uint8 BEHAVIOR_FREEZE = 1 # All joints frozen in current position.
uint8 BEHAVIOR_STAND_PREP = 2 # Robot assumes a balanced posture, preparing for stand.
uint8 BEHAVIOR_STAND = 3 # Statically stable stand.
uint8 BEHAVIOR_WALK = 4 # Dynamically stable walk.
uint8 BEHAVIOR_STEP = 5 # Statically stable, single-step walk.
uint8 BEHAVIOR_MANIPULATE = 6 # Statically stable stand, with upper body joints controlled by user.
uint8 BEHAVIOR_USER = 7 # Setpoints and gains set by user.
uint8 BEHAVIOR_CALIBRATE = 8 # Runs calibration
uint8 NUM_BEHAVIORS = 9

