# Status of robot, environment, and behaviors for use by external AI

# Pose of the robot.
# Mid point between footsoles and facing the direction of the pelvis in world frame.
# Z always points straight up.
geometry_msgs/Pose robot_mid_feet_under_pelvis_pose_in_world

# List of objects in the scene
behavior_msgs/AI2RObjectMessage[] objects

# List of available behaviors (JSON file names)
string[] available_behaviors


