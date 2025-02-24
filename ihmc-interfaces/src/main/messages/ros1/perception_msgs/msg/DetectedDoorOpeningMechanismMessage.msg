# Message representing the DoorOpeningMechanism

string name

# Ordinal of DoorSide
int8 side

# Pose of the opening mechanism.
# May contain NaN in the position and/or orientation values if unknown.
geometry_msgs/Pose pose


