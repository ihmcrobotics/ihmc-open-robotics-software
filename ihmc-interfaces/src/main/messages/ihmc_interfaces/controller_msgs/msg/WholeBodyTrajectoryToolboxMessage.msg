# Message that carries all the information required to configure the IHMC whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

WholeBodyTrajectoryToolboxConfigurationMessage configuration
WaypointBasedTrajectoryMessage[<=10] end_effector_trajectories
RigidBodyExplorationConfigurationMessage[<=10] exploration_configurations
ReachingManifoldMessage[<=10] reaching_manifolds