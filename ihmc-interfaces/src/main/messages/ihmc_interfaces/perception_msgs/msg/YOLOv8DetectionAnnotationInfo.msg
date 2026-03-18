# Information used for annotating a single YOLOv8 detection

# Time the detection was made
ihmc_common_msgs/InstantMessage detection_instant

# Class of the detected object
string object_class

# Bounding box of the detection
vision_msgs/BoundingBox2D bounding_box

Point2DArray[] mask_polygons
