# mirrored from AtlasControlTypes.h
uint32 STEP_SUBSTATE_SWAYING = 0  # Feet are in double support. This flag does not latch. Only one of STEP_SUBSTATE_SWAYING or STEP_SUBSTATE_STEPPING will be set at any given time.
uint32 STEP_SUBSTATE_STEPPING = 1 # Actively stepping; one foot is in the air. This flag does not latch.

uint32 status_flags    # use AtlasBeahviorFeedback/status_flags
float64 t_step_rem
uint32 current_step_index
uint32 next_step_index_needed
atlas_msgs/AtlasBehaviorStepData desired_step_saturated
