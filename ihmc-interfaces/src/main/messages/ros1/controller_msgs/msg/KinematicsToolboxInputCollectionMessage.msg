# This message is part of the IHMC whole-body inverse kinematics module.
# This message allows to pack and send as a single message a collection of inputs for the kinematics toolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Inputs for controlling the center of mass position.
controller_msgs/KinematicsToolboxCenterOfMassMessage[] center_of_mass_inputs

# Inputs for controlling rigid-bodies.
controller_msgs/KinematicsToolboxRigidBodyMessage[] rigid_body_inputs

# Inputs for controlling 1-DoF joints.
controller_msgs/KinematicsToolboxOneDoFJointMessage[] joint_inputs

# Input for overriding the default support polygon with a custom one.
controller_msgs/KinematicsToolboxSupportRegionMessage[] contact_state_input


