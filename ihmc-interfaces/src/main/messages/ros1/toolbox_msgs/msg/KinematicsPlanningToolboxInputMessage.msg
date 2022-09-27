
# This message is part of the IHMC whole-body inverse kinematics module: KinematicsPlanningToolbox.
# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# This is a list of rigid body messages which is used in KinematicsPlanningToolbox.
# It should composed of at least one message. Each message represent the input for solving the trajectory for one rigid-body.
toolbox_msgs/KinematicsPlanningToolboxRigidBodyMessage[] rigid_body_messages

# This is desired key frames for center of mass position and will be used in KinematicsPlanningToolbox.
# In case this message is empty, the kinematics solver will keep center of mass position in its current location.
toolbox_msgs/KinematicsPlanningToolboxCenterOfMassMessage center_of_mass_message

# This is kinematics configuration message and will configure whole body inverse kinematics solver.
# This is optional.
toolbox_msgs/KinematicsToolboxConfigurationMessage kinematics_configuration_message


