# Message representing a DetectedDoor

# UUID of this detection
ihmc_common_msgs/UUIDMessage detection_uuid

# The opening mechanism
perception_msgs/DetectedDoorOpeningMechanismMessage opening_mechanism

# Panel pose
# May contain NaN in the position and/or orientation values if unknown.
controller_msgs/RigidBodyTransformMessage panel_transform_to_world

# Planar region of the panel
perception_msgs/PlanarRegionMessage panel_planar_region

# Last detection instant
ihmc_common_msgs/InstantMessage last_detection_time

# Internal statistics
ihmc_common_msgs/InstantMessage opening_mechanism_first_detection_time

int32 opening_mechanism_detection_count

ihmc_common_msgs/InstantMessage panel_first_detection_time

int32 panel_detection_count


