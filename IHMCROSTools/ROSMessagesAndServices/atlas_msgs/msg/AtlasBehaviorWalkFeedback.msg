# mirrored from AtlasControlTypes.h
float64 t_step_rem
uint32 current_step_index
uint32 next_step_index_needed
uint32 status_flags    # use AtlasBeahviorFeedback/status_flags
atlas_msgs/AtlasBehaviorStepData[4] step_queue_saturated # 4 is hardcoded in AtlasSimInterface library.
