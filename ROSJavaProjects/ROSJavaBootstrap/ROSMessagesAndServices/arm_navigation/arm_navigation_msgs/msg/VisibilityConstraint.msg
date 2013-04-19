# This message contains the definition of a visibility constraint.
Header header

# The point stamped target that needs to be kept within view of the sensor
geometry_msgs/PointStamped target

# The local pose of the frame in which visibility is to be maintained
# The frame id should represent the robot link to which the sensor is attached
# The visual axis of the sensor is assumed to be along the X axis of this frame
geometry_msgs/PoseStamped sensor_pose

# The deviation (in radians) that will be tolerated
# Constraint error will be measured as the solid angle between the 
# X axis of the frame defined above and the vector between the origin 
# of the frame defined above and the target location
float64 absolute_tolerance

