# Parameters for running YOLOv8

# Confidence required to consider a detection valid
float32 confidence_threshold

float32 non_maximum_suppression_threshold

float32 segmentation_threshold

# DetectionFilter stability threshold
float32 candidate_acceptance_threshold

# How much to shrink the YOLO mask
int32 erosion_kernel_radius

# Rejection threshold for outlier points in segmented point cloud of the object
float32 outlier_rejection_threshold

# Maximum distance between new and old detections to be considered the same object
float32 match_distance_threshold

# List of YOLOv8DetectionClass values that YOLO should detect
int8[] target_detection_classes


