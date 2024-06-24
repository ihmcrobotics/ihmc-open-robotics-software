# Message representing an InstantDetection of any type.

# Standard InstantDetection fields:
string detected_object_class
string detected_object_name
float64 confidence
geometry_msgs/Pose object_pose
ihmc_common_msgs/InstantMessage detection_time
ihmc_common_msgs/UUIDMessage persistent_detection_id

# YOLOv8InstantDetection fields:
geometry_msgs/Point32[<=511] yolo_object_point_cloud

# CenterPoseInstantDetection fields:
geometry_msgs/Point[8] center_pose_bounding_box_2d_vertices
geometry_msgs/Point[8] center_pose_bounding_box_vertices
