# Structure for returning errors and other information related to behaviors.
uint32 status_flags # Bit-wise array of AtlasBehaviorFlags values indicating non-specific-behavior status.
int32 trans_from_behavior_index # Index of the behavior from which a transition is being attempted.
int32 trans_to_behavior_index # Index of the behavior to which a transition is being attempted.
