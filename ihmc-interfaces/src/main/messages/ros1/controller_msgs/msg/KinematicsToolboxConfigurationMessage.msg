# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Indicates whether the privileged_root_joint_position is to be used or not.
bool use_privileged_root_joint_position

# Indicates whether the privileged_root_joint_orientation is to be used or not.
bool use_privileged_root_joint_orientation

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
# Field default value -1.0
float64 privileged_weight

# The feedback proportional gain to use for the privileged configuration.
# It is coupled to some extent to the privileged_weight
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 privileged_gain

# Specifies how much high joint velocity values should be penalized in the optimization problem.
# A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 joint_velocity_weight

# Specifying how much high joint acceleration values should be penalized in the optimization problem.
# A value of -1 will result in the solver using its default value.
# Field default value -1.0
float64 joint_acceleration_weight

# When true, the solver will enforce the joint velocity limits as defined in the robot model.
# Enabling this restriction will augment the number of iteration before converging to a robot configuration for a given set of end-effector positions.
bool enable_joint_velocity_limits

# When true, the solver will ignore the joint velocity limits.
# Enabling this restriction will reduce the number of iteration before converging to a robot configuration for a given set of end-effector positions.
bool disable_joint_velocity_limits

# If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
# In case it has undesirable effects, use this flag to disable it.
bool disable_collision_avoidance

# In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
bool enable_collision_avoidance


