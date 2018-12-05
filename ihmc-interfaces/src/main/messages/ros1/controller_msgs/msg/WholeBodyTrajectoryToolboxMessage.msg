# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

controller_msgs/WholeBodyTrajectoryToolboxConfigurationMessage configuration

controller_msgs/WaypointBasedTrajectoryMessage[] end_effector_trajectories

controller_msgs/RigidBodyExplorationConfigurationMessage[] exploration_configurations

controller_msgs/ReachingManifoldMessage[] reaching_manifolds


