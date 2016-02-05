# A trajectory message that encodes joint limits within it.
trajectory_msgs/JointTrajectory trajectory

# A vector of JointLimit messages.
# Each message contains the limits for a specific joint
arm_navigation_msgs/JointLimits[] limits
