# Defines a 2D detection result.
# 
# This is similar to a 2D classification, but includes position information,
# allowing a classification result for a specific crop or image point to
# to be located in the larger image.

Header header

# Class probabilities
vision_msgs/ObjectHypothesisWithPose[] results

# 2D bounding box surrounding the object.
vision_msgs/BoundingBox2D bbox

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id


