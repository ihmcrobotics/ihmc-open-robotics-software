# This message is part of the IHMC robot environment awareness module (REA).
# It is destined to gather complementary information related to the current state REA is in.

# Informs if REA is currently collecting sensor data and estimating planar regions.
# Note that if both, is_using_lidar and is_using_stereo_vision are false, REA is running but not collecting data.
bool is_running false
# Informs whether REA is collecting LIDAR data or not.
bool is_using_lidar false
# Informs whether REA is collecting pointcloud data from a stereo-camera.
bool is_using_stereo_vision false
# Informs whether REA's internal data has just been cleared.
bool has_cleared false

# Provides the current set of filter parameters used in REA.
REASensorDataFilterParametersMessage current_sensor_filter_parameters