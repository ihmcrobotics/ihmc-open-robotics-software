# Enum identifying possible flags from the walk behavior
uint32 flag

uint32 WALK_OKAY                            = 0
uint32 WALK_SUBSTATE_SWAYING                = 1
uint32 WALK_SUBSTATE_STEPPING               = 2
uint32 WALK_SUBSTATE_CATCHING               = 4
uint32 WALK_WARNING_INSUFFICIENT_STEP_DATA  = 8
uint32 WALK_ERROR_INCONSISTENT_STEPS        = 16
