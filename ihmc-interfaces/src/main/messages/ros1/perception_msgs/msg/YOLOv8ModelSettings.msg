# Settings for an individual YOLOv8 Model
# Below are parameters not used directly by the YOLOv8Model class,
# but related to the way we use the YOLO output.

string model_name

# A boolean for each object class, whether its detections should be ignored
bool[] ignored_object_classes

# Confidence thresholds, corresponding to each detectable object class
float32[] confidence_thresholds

# Mask thresholds, corresponding to each detectable object class
float32[] mask_thresholds

# Non-maximum suppression threshold
float32 non_maximum_suppression_threshold

# Radii used for mask erosion, corresponding to each detectable object class
uint16[] erosion_kernel_radii

# Thresholds for removing outlier points of segmented point cloud
float32[] outlier_thresholds


