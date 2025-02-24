# Message representing a DetectedDoor

# The opening mechanism
perception_msgs/DetectedDoorOpeningMechanismMessage opening_mechanism

# Panel pose
# May contain NaN in the position and/or orientation values if unknown.
geometry_msgs/Pose panel_pose

# Planar region of the panel
perception_msgs/PlanarRegionMessage panel_planar_region

# Last detection instant
ihmc_common_msgs/InstantMessage last_detection_time


