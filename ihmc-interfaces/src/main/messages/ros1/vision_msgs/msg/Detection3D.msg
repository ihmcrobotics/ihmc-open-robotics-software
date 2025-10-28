# Defines a 3D detection result.
# 
# This extends a basic 3D classification by including the pose of the
# detected object.

Header header

# Class probabilities. Does not have to include hypotheses for all possible
# object ids, the scores for any ids not listed are assumed to be 0.
vision_msgs/ObjectHypothesisWithPose[] results

# 3D bounding box surrounding the object.
vision_msgs/BoundingBox3D bbox

# ID used for consistency across multiple detection messages. Detections
# of the same object in different detection messages should have the same id.
# This field may be empty.
string id


