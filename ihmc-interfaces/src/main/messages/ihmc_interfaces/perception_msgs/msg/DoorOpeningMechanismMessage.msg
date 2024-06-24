# The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
byte type

# The side of the door the opening mechanism is mounted to
# 0 PUSH
# 1 PULL
byte door_side

# The grasp pose of the opening mechanism
geometry_msgs/Pose grasp_pose
