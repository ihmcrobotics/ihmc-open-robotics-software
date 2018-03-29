# This message is part of the IHMC whole-body controller API.
# This message can be used to send a predefined angular momentum trajectory to the controller.
# This trajectory will be used for ICP planning.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# List of angular momentum trajectory waypoints.
# Each waypoint contains the angular momentum and the angular momentum rate at a given time.
# NOTE: As the angular momentum can't be encoded using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used. 
EuclideanTrajectoryMessage angular_momentum_trajectory
