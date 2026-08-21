# General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
# Main usage is for the IHMC KinematicsToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# Nothing reported by the toolbox.
byte CURRENT_TOOLBOX_STATE_NO_STATUS = 0
# The toolbox just initialized successfully and is about to start running.
byte CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL = 1
# The toolbox failed its initialization and cannot run until it succeeds.
# This failure specifies that the toolbox has not received RobotConfigurationData from the IHMC walking controller.
byte CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD = 2
# The toolbox has been initialized properly and is running.
byte CURRENT_TOOLBOX_STATE_RUNNING = 3

# Provides insight about the current state of the toolbox, e.g. waiting for user input or waiting for controller input.
byte current_toolbox_state 0

int32 joint_name_hash

# Desired joint positions.
float32[] desired_joint_angles

# Desired root joint position in world frame.
geometry_msgs/Point desired_root_position
# Desired root joint orientation in world frame.
geometry_msgs/Quaternion desired_root_orientation

# Desired joint velocities.
float32[] desired_joint_velocities

# Desired linear velocity of the root joint expressed in local frame.
geometry_msgs/Vector3 desired_root_linear_velocity
# Desired angular velocity of the root joint expressed in local frame.
geometry_msgs/Vector3 desired_root_angular_velocity

# Support region used by the toolbox
geometry_msgs/Point[<=32] support_region

# Desired joint velocities derived from finite difference from positions at publishing period.
float32[] desired_joint_velocities_publishing_period
# Desired linear velocity of the root joint expressed in local frame, derived from finite difference from positions at publishing period.
geometry_msgs/Vector3 desired_root_linear_velocity_publishing_period
# Desired angular velocity of the root joint expressed in local frame, derived from finite difference from positions at publishing period.
geometry_msgs/Vector3 desired_root_angular_velocity_publishing_period

geometry_msgs/Point desired_torso_position
geometry_msgs/Quaternion desired_torso_orientation

float32 com_offset

# Legged robot-specific contact information (false if not a legged robot)
bool left_foot_in_contact
bool right_foot_in_contact

float64 solution_quality -1

# True on the last logged frame of a kinematics streaming replay. Missing in old JSON logs means false.
bool end_replay false