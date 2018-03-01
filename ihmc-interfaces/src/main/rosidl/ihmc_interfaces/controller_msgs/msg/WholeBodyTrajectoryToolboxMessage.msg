# Message that carries all the information required to configure the IHMC whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

WholeBodyTrajectoryToolboxConfigurationMessage configuration
WaypointBasedTrajectoryMessage[<=10] end_effector_trajectories
RigidBodyExplorationConfigurationMessage[<=10] exploration_configurations
ReachingManifoldMessage[<=10] reaching_manifolds