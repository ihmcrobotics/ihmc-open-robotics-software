# Detection timestamp. Should be the acquisition time of the image used for detection
ihmc_common_msgs/InstantMessage timestamp

# ID of the detected object
string object_id

# Mesh file name of the object (to identify it's class)
string mesh_file

# Pose of the object
geometry_msgs/Pose object_pose