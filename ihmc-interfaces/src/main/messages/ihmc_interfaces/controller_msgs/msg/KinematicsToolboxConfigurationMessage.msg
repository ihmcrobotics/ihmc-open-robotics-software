# This message is part of the IHMC whole-body inverse kinematics module.
# It contains auxiliary information that allows to further customized the behavior of the solver.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Indicates whether the privileged_root_joint_position is to be used or not.
bool use_privileged_root_joint_position false
# Indicates whether the privileged_root_joint_orientation is to be used or not.
bool use_privileged_root_joint_orientation false
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Point privileged_root_joint_position
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
geometry_msgs/Quaternion privileged_root_joint_orientation
# This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
# See Joint.hashCode() for the computation of a joint hash code.
int32[] privileged_joint_hash_codes
# When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
float32[] privileged_joint_angles
# The weight to use in the optimization for the privileged configuration.
# When remaining close to the privileged configuration is important, raise this weight to a value higher than the 
# weight of the main objectives.
# Any value less than zero will be ignored.
# A value of -1 will result in the solver using its default value.
float64 privileged_weight -1
# The feedback proportional gain to use for the privileged configuration.
# It is coupled to some extent to the privileged_weight
# A value of -1 will result in the solver using its default value.
float64 privileged_gain -1

# Specifies how much high joint velocity values should be penalized in the optimization problem.
# A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
# A value of -1 will result in the solver using its default value.
float64 joint_velocity_weight -1
# Specifying how much high joint acceleration values should be penalized in the optimization problem.
# A value of -1 will result in the solver using its default value.
float64 joint_acceleration_weight -1

# If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
# In case it has undesirable effects, use this flag to disable it.
bool disable_collision_avoidance false
# In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
bool enable_collision_avoidance false