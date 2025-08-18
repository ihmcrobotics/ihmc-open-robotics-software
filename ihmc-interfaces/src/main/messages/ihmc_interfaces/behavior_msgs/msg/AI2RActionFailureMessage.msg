# For use by external AI to modify the behavior tree and operate behaviors

# Name of the action that has failed
string action_name

# Type of the action
string action_type

# Reference frame of the action
string action_frame

# Whether the reference frame of the failed action was missing
bool missing_frame

# Name of the object the action would collide with in not interrupted (marked as failed)
string collision_name

# Position tolerance for the action [m]
float64 position_tolerance
# Orientation tolerance for the action [deg]
float64 orientation_tolerance

# Error in position for the action
geometry_msgs/Point position_error
# Error in orientation for the action
geometry_msgs/Quaternion orientation_error