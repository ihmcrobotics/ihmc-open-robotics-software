# Settings for the YOLOv8DetectionExecutor class

ihmc_common_msgs/LatestModificationMessage latest_timestamp_modifiable

# List of models to run
string[<=16] models_to_run

# Settings for each model
perception_msgs/YOLOv8ModelSettings[<=16] model_settings
