# Nothing reported by the toolbox.
int8 CURRENT_TOOLBOX_STATE_NO_STATUS=0

# The toolbox just initialized successfully and is about to start running.
int8 CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL=1

# The toolbox failed its initialization and cannot run until it succeeds.
# This failure specifies that the toolbox has not received RobotConfigurationData from the IHMC walking controller.
int8 CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD=2

# The toolbox has been initialized properly and is running.
int8 CURRENT_TOOLBOX_STATE_RUNNING=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Provides insight about the current state of the toolbox, e.g. waiting for user input or waiting for controller input.
int8 current_toolbox_state

int32 joint_name_hash

float32[] desired_joint_angles

geometry_msgs/Vector3 desired_root_translation

geometry_msgs/Quaternion desired_root_orientation

# Desired joint velocities might be empty.
float32[] desired_joint_velocities

# Desired twist of root might be empty.
geometry_msgs/Vector3 desired_root_linear_velocity

geometry_msgs/Vector3 desired_root_angular_velocity

# Field default value -1.0
float64 solution_quality


