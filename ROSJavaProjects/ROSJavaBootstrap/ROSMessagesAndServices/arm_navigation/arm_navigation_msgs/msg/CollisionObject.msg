# a header, used for interpreting the poses
Header header

# the id of the object
string id

# The padding used for filtering points near the object.
# This does not affect collision checking for the object.  
# Set to negative to get zero padding.
float32 padding

#This contains what is to be done with the object
CollisionObjectOperation operation

#the shapes associated with the object
arm_navigation_msgs/Shape[] shapes

#the poses associated with the shapes - will be transformed using the header
geometry_msgs/Pose[] poses
