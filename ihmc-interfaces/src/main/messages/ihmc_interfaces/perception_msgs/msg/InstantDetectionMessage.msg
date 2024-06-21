# Message representing an InstantDetection of any type.

# Standard InstantDetection fields:
string detected_object_class
string detected_object_name
float64 confidence
geometry_msgs/Pose object_pose
ihmc_common_msgs/InstantMessage detection_time

# YOLOv8InstantDetection fields:
geometry_msgs/Point32[<=5000] yolo_object_point_cloud

# CenterPoseInstantDetection fields:
geometry_msgs/Point[8] center_pose_bounding_box_2d_vertices
geometry_msgs/Point[8] center_pose_bounding_box_vertices
