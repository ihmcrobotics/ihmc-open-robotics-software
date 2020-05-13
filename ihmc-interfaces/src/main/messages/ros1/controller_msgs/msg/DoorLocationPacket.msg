


# This message is part of the IHMC humanoid behavior module.



















# Unique ID used to identify this message, should preferably be consecutively increasing.



uint32 sequence_id





geometry_msgs/Pose door_transform_to_world





# Some behaviors will average the door location to remove noise,

# trustedPosition allows someone to tell the behavior that this is an accurate location and and can does not need to be averaged,

# this is useful for sending door locations manually from the ui*/



bool trusted_position



