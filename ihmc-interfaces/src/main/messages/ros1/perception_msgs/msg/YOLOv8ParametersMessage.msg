# TODO: Having these parameter/settings messages kinda sucks.
# Should try to get a better system.
# Parameters for running YOLOv8

# Confidence required to consider a mask a valid detection
float32 confidence_threshold

float32 non_maximum_suppression_threshold

float32 segmentation_threshold

# How much to shrink the object mask. Higher value = more shrinkage
int32 erosion_kernel_radius

float32 outlier_threshold

# List of YOLOv8DetectionClass values that YOLO should detect
int8[] target_detection_classes


