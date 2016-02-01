# Potential values for task_type
uint32 TASK_OTHER = 0
uint32 TASK_DRIVING = 1
uint32 TASK_WALKING = 2
uint32 TASK_MANIPULATION = 3
# Absolute wall time
time wall_time
# Absolute sim time
time sim_time
# Wall time elapsed since passing the first gate
time wall_time_elapsed
# Sim time elapsed since passing the first gate
time sim_time_elapsed
# How many parts of the task have been accomplished (e.g., gates traversed)
int32 completion_score
# How many damaging falls detected
int32 falls
# Optional message to describe events
string message
# Which type of task
uint32 task_type
