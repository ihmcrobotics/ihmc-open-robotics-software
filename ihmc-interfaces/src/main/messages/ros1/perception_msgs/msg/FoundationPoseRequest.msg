
# ID that will be assigned to the object
string object_id

# Name of the mesh file for the object
string mesh_file

# Color image containing the object
perception_msgs/ImageMessage color

# Depth image containing the object
perception_msgs/ImageMessage depth

# Mask of the object in the image
perception_msgs/ImageMessage object_mask


