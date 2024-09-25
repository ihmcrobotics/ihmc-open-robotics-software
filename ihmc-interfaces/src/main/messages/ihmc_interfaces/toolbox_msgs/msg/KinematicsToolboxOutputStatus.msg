# General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
# Main usage is for the IHMC KinematicsToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Nothing reported by the toolbox.
byte CURRENT_TOOLBOX_STATE_NO_STATUS = 0
# The toolbox just initialized successfully and is about to start running.
byte CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL = 1
# The toolbox failed its initialization and cannot run until it succeeds.
# This failure specifies that the toolbox has not received RobotConfigurationData from the IHMC walking controller.
byte CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD = 2
# The toolbox has been initialized properly and is running.
byte CURRENT_TOOLBOX_STATE_RUNNING = 3

# Low margin, high sensitivity
byte POSTURE_OPTIMIZER_MODE_OPTIMIZER = 0
# Medium margin or low margin + low sensitivity
byte POSTURE_OPTIMIZER_MODE_FREEZE = 1
# High margin
byte POSTURE_OPTIMIZER_MODE_NOMINAL = 2

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

# Multi-contact feasible com region computed by the toolbox (if upper body is load-bearing)
geometry_msgs/Point[<=18] multi_contact_feasible_com_region
# Lowest-proximity edge index in multi-contact com region (if computed)
int32 closest_edge_index -1
# CoM stability margin if upper body is load-bearing
float64 center_of_mass_stability_margin -1.0
# CoM stability margin sensitivity with respect to the robot's posture
float64 com_stability_posture_sensitivity -1.0

byte posture_optimizer_state 2

float64 solution_quality -1