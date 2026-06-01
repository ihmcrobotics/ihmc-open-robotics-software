# Information used for annotating a single YOLOv8 detection

# Class of the detected object
string object_class

# Confidence of the detection
float32 confidence

# Bounding box of the detection
vision_msgs/BoundingBox2D bounding_box

# Polygons representing the mask of the detection
Float32MultiArrayHack mask_polygons

# TODO: jros2 replace above with below
# std_msgs/Float32MultiArray mask_polygons
