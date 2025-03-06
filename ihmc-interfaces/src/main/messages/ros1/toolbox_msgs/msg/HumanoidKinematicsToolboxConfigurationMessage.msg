# This message is part of the IHMC whole-body inverse kinematics module.
# Similar to KinematicsToolboxConfigurationMessage, this contains auxiliary information that allows to further customized the behavior of the solver.
# The parameters exposed through this message are specific to application to humanoid robots.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint64 sequence_id

# When set to true, the solver will maintain, if possible, the current x and y coordinates of the center
# of mass. By 'current', it means that the solver will use the robot configuration data
# broadcasted by the controller to obtain the center of mass position.
# Field default value True
bool hold_current_center_of_mass_xy_position

# When set to true, the solver will process the balance status from an active controller session and build a support polygon.
# Note that the auto support polygon feature is overidden by a user specified support polygon when provided.
# - when the walking controller is running, the CapturabilityBasedStatus message is used to identify the current support polygon.
# - when the multi-contact controller is running, the MultiContactBalanceStatus message is used to identify the current support polygon.
# Field default value True
bool enable_auto_support_polygon

# If true, enables an objective for improving the stability margin of the robot when the upper body is load-bearing.
bool enable_stability_objective

# If true, enables a preview of the feasible region and corresponding contact-point adjustment.
bool enable_contact_adjustment

# If enable_contact_adjustment is true this the origin position of the frame describing the upcoming bracing region
geometry_msgs/Point region_point

# If enable_contact_adjustment is true this the origin orientation of the frame describing the upcoming bracing region. The normal is along z in the given plane
geometry_msgs/Quaternion region_orientation

# Normal in world. This is over-specific but available to use if necessary
geometry_msgs/Vector3 region_normal

# If enable_contact_adjustment is true this the convex hull of the upcoming bracing region
ihmc_common_msgs/Point2DMessage[] region_vertices

# Whether restrictive joint limits are enabled, in order to have the IK avoid a solution at the joint limit.
# Field default value True
bool enable_joint_limit_reduction

# By default, the hip joint limits are restricted by 0.05 of the RoM.
# When this and the subsequent list are set, the default limit restrictions are replaced with these values.
# This list are the new joint limit reduction factors to be used, such that a value 0.05 means the restricted RoM will be 0.95 of the nominal (0.025 from either end).
float32[] joint_limit_reduction_factors

# The list of joints that the field joint_limit_reduction_values correspond to, by hash-code. The hash-code is computed from OneDoFJoint#hashcode().
int32[] joint_limit_reduction_hash_codes


