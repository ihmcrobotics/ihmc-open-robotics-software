# Parameters for running YOLOv8

# Confidence required to consider a mask a valid detection
float32 confidence_threshold

float32 non_maximum_suppression_threshold

float32 segmentation_threshold

# Percentage of updates a candidate must be detected in to be accepted
float32 candidate_acceptance_threshold

# List of YOLOv8DetectionClass values that YOLO should detect
byte[<=100] target_detection_classes