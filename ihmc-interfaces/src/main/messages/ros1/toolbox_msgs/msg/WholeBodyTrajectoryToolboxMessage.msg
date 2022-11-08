# Message that carries all the information required to configure the IHMC whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

toolbox_msgs/WholeBodyTrajectoryToolboxConfigurationMessage configuration

toolbox_msgs/WaypointBasedTrajectoryMessage[] end_effector_trajectories

toolbox_msgs/RigidBodyExplorationConfigurationMessage[] exploration_configurations

toolbox_msgs/ReachingManifoldMessage[] reaching_manifolds


