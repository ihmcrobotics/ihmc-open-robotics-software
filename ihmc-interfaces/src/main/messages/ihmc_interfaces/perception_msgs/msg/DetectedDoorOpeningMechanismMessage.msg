# Message representing the DoorOpeningMechanism

string name

# Ordinal of DoorSide
byte side

# Pose of the opening mechanism.
# May contain NaN in the position and/or orientation values if unknown.
controller_msgs/RigidBodyTransformMessage transform_to_world
