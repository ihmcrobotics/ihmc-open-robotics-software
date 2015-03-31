# Structure for feedback data specific to the Walk behavior.
float64 t_step_rem # Estimated step time remaining.
int32 current_step_index # Current step index.
int32 next_step_index_needed # Next step index needed.
atlas_hardware_interface/AtlasBehaviorWalkFlags status_flags # Bit-wise array of AtlasBehaviorWalkFlags values indicating Walk behavior status.
atlas_hardware_interface/AtlasBehaviorStepData[4] step_queue_saturated # Saturated copy of step_queue from user.
