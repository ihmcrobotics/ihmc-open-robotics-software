# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When false, the toolbox will only publish a status with KinematicsToolboxOutputStatus such that the user can validate
# that the solver is working properly.
# When true, the toolbox will stream to the IHMC walking controller the desired robot configuration. The status mentioned is
# still published so the user can compare the solver desired state against the actual robot state.
bool stream_to_controller

# The list of inputs the solver is to be tracking.
# When streaming inputs from a VR UI environment, it is convenient to use the fields control_frame_position_in_end_effector and
# control_frame_orientation_in_end_effector from KinematicsToolboxRigidBodyMessage to adjust the user's controllers with respect
# to the robot's end-effectors.
controller_msgs/KinematicsToolboxRigidBodyMessage[] inputs


