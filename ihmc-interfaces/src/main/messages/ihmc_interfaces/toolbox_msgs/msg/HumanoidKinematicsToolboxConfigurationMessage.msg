# This message is part of the IHMC whole-body inverse kinematics module.
# Similar to KinematicsToolboxConfigurationMessage, this contains auxiliary information that allows to further customized the behavior of the solver.
# The parameters exposed through this message are specific to application to humanoid robots.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
# of mass. By 'current', it means that the solver will use the robot configuration data
# broadcasted by the controller to obtain the center of mass position.
bool hold_current_center_of_mass_xy_position true
# When set to true, the solver will process the balance status from an active controller session and build a support polygon.
# Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
# - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
# - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
bool enable_auto_support_polygon true
# When set to true, the solver will hold the pose of the rigid-bodies with active contact points.
# - when the walking controller is running, the rigid-bodies in question are the feet.
# - when a multi-contact controller is running, any rigid-body of the robot can be in contact.
bool hold_support_rigid_bodies true
# If this is true and the solver receives a MultiContactBalanceStatus, it will solve for the multi-contact support region
bool enable_multi_contact_support_region_solver false

# Whether restrictive joint limits are enabled, in order to have the IK avoid a solution at the joint limit.
bool enable_joint_limit_reduction true
# By default, the hip joint limits are restricted by 0.05 of the RoM.
# When this and the subsequent list are set, the default limit restrictions are replaced with these values.
# This list are the new joint limit reduction factors to be used, such that a value 0.05 means the restricted RoM will be 0.95 of the nominal (0.025 from either end).
float32[<=20] joint_limit_reduction_factors
# The list of joints that the field joint_limit_reduction_values correspond to, by hash-code. The hash-code is computed from OneDoFJoint#hashcode().
int32[<=20] joint_limit_reduction_hash_codes