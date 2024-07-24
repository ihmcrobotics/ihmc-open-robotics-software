# This message is part of the IHMC whole-body inverse kinematics module.
# It contains auxiliary information that allows to further customized the behavior of the solver.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies how much high joint velocity values should be penalized in the optimization problem.
# A low value generally results in a reduce number of iterations before convergence but it also decreases the general stability of the solver.
# A value of -1 will result in the solver using its default value.
float64 joint_velocity_weight -1
# Specifying how much high joint acceleration values should be penalized in the optimization problem.
# A value of -1 will result in the solver using its default value.
float64 joint_acceleration_weight -1
# When true, the solver will enforce the joint velocity limits as defined in the robot model.
# Enabling this restriction will augment the number of iteration before converging to a robot configuration for a given set of end-effector positions.
bool enable_joint_velocity_limits false
# When true, the solver will ignore the joint velocity limits.
# Enabling this restriction will reduce the number of iteration before converging to a robot configuration for a given set of end-effector positions.
bool disable_joint_velocity_limits false

# If the toolbox has been setup with the collision model of the robot, it will by default handle self-collision avoidance.
# In case it has undesirable effects, use this flag to disable it.
bool disable_collision_avoidance false
# In case collision avoidance has been disabled, use this flag to re-enable it while leaving disable_collision_avoidance to false.
bool enable_collision_avoidance false

# By default, the kinematics toolbox will preserve all input that has been provided until it is reinitialized.
# For instance, if the user sends a command for controlling a certain body, the command will persist even if the user stops sending it.
# A command that persists is overridden if another is received for the same body.
# Priviliged configuration inputs are not affected by this flag.
# In case persistence of inputs is not desired, use this flag to disable it.
# When persistence is disabled, it is strongly recommended to use KinematicsToolboxInputCollectionMessage to send inputs to the toolbox.
bool disable_input_persistence false
# In case input persistence has been disabled, use this flat to re-enable it while leaving disable_input_persistence to false.
bool enable_input_persistence false

# When true, the solver enforces the solution to have the projection of the center of mass contained
# inside the current support polygon. The support polygon can be determined automatically using the controller's ouput if running.
# The support polygon can also be defined by the user by sending a KinematicsToolboxContactStateMessage.
bool enable_support_polygon_constraint
# When true, this disables the support polygon constraint on the center of mass.
bool disable_support_polygon_constraint

# Hash codes of the joints to be deactivated in the solver
int32[<=10] joints_to_deactivate
# Hash codes of the joints to be re-activated in the solver
int32[<=10] joints_to_activate

