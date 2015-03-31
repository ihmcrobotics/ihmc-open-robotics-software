# Structure for feedback data specific to the Step behavior.
float64 t_step_rem # Estimated step time remaining before new step data is needed.
int32 current_step_index # Current step index.
int32 next_step_index_needed # Next step index needed.
atlas_hardware_interface/AtlasBehaviorStepFlags status_flags # Bit-wise array of AtlasBehaviorStepFlags values indicating Step behavior status.
atlas_hardware_interface/AtlasBehaviorStepData desired_step_saturated # Saturated copy of desired_step from user.
