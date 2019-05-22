# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Defines the list of steps to perform
QuadrupedTimedStepMessage[] quadruped_step_list
# If true, the times are expressed in absolute time relative to the robot start
bool is_expressed_in_absolute_time true
# Properties for queueing footstep lists.
QueueableMessage queueing_properties
# Defines if the step list is adjustable
bool are_steps_adjustable true
# If true the controller will adjust the z coordinate of the upcoming footsteps with the location error of previous steps.
bool offset_steps_height_with_execution_error false