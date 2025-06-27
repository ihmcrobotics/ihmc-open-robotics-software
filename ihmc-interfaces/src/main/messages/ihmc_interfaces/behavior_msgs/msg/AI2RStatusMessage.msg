# Status of robot, environment, and behaviors for use by external AI

# Pose of the robot.
# Mid point between footsoles and facing the direction of the pelvis in world frame.
# Z always points straight up.
geometry_msgs/Pose robot_mid_feet_under_pelvis_pose_in_world

# List of objects in the scene
behavior_msgs/AI2RObjectMessage[<=200] objects

# List of available behaviors (checkpoints from a pre-loaded behavior)
string[<=200] available_behaviors

# Name of the behavior that is in progress
string behavior_in_progress

# Name of the behavior that has succeeded (name of the checkpoint in the pre-loaded behavior collection)
string completed_behavior

# Name of the behavior that has failed (name of the checkpoint in the pre-loaded behavior collection)
string failed_behavior
behavior_msgs/AI2RActionFailureMessage failure

# Name of the object that the robot is handling
string object_grasped

# Which hand is handling the object
byte grasp_side 255

# Relative transform of the grasped object in hand frame
geometry_msgs/Pose transform_grasped_object_hand

# Name of the object that the robot has released
string object_placed
