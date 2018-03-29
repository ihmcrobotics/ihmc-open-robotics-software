# Message that carries all the information required to configure the IHMC whole-body trajectory planner.
# Main usage is the IHMC WholeBodyTrajectoryToolbox.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

WholeBodyTrajectoryToolboxConfigurationMessage configuration
WaypointBasedTrajectoryMessage[<=10] end_effector_trajectories
RigidBodyExplorationConfigurationMessage[<=10] exploration_configurations
ReachingManifoldMessage[<=10] reaching_manifolds