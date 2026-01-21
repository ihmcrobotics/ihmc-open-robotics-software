# A behavior tree scene object

# The timestamp and modifier ID of the latest modification to this object's data fields
ihmc_common_msgs/LatestModificationMessage latest_modification_to_data

# The object's unique ID
uint32 id

behavior_msgs/BehaviorTreeSceneObjectDefinitionMessage definition

# Persistent detection associated with this object
behavior_msgs/PersistentDetectionStatusMessage persistent_detection

# Transform of the object frame to world frame
controller_msgs/RigidBodyTransformMessage transform_to_world

# Used only for door panel
behavior_msgs/PersistentDetectionStatusMessage door_panel_detection
