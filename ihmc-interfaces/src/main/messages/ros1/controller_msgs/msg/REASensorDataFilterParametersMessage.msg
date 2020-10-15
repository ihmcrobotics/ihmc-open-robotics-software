# This message is part of the IHMC robot environment awareness module (REA).
# This provides access to a few filters used in REA.

# Lower-bound of the bounding box inside which sensor data will be processed into planar regions.
# Note that the coordinates are relative to the sensor position and yaw:
# - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
# - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
# - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
# Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
# If any of the lower-bound coordinates is NaN, the bounding box parameters will be ignored.
geometry_msgs/Point bounding_box_min

# Upper-bound of the bounding box inside which sensor data will be processed into planar regions.
# Note that the coordinates are relative to the sensor position and yaw:
# - x-coordinate: represents the distance in front (positive) or in the back (negative) of the sensor.
# - y-coordinate: represents the distance to the left (positive) or to the right (negative) of the sensor.
# - z-coordinate: represents the distance above (positive) or below (negative) of the sensor.
# Sensor data collected outside this bounding box is still used to help estimating empty space inside it.
# If any of the upper-bound coordinates is NaN, the bounding box parameters will be ignored.
geometry_msgs/Point bounding_box_max

# Defines a bounding sphere centered at the sensor within which sensor data will be processed into planar regions.
# Sensor data collected outside this bounding sphere is still used to help estimating empty space inside it.
# A negative value will be ignored.
float64 sensor_max_range

# Defines a bounding sphere centered at the sensor within which sensor data is to be ignored.
# A negative value will be ignored.
float64 sensor_min_range


