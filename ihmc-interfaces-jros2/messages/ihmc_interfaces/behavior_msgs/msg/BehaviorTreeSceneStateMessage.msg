# The state of the behavior tree scene

# The timestamp and modifier ID of the latest modification to the list of objects
ihmc_common_msgs/LatestModificationMessage latest_modification_to_list

# Scene objects
behavior_msgs/BehaviorTreeSceneObjectStateMessage[<=100] objects

# Persistent detections tracked in the scene
behavior_msgs/PersistentDetectionStatusMessage[<=100] persistent_detections

float32 pose_filter_alpha
float32 acceptance_confidence
float32 stability_frequency
float32 history_duration
