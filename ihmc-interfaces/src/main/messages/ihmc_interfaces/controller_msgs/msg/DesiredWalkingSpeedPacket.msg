# This message is part of the IHMC fast walking controller API.
# This message sets the desired walking and turning speed.
# This message may initiate or stop walking.

# Define the desired 3D linear velocity to be reached.
geometry_msgs/Vector3 desired_speed
# Define the desired turning velocity
float64 turning_speed

