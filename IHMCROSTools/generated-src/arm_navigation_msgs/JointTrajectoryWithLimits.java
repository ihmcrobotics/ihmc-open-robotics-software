package arm_navigation_msgs;

public interface JointTrajectoryWithLimits extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/JointTrajectoryWithLimits";
  static final java.lang.String _DEFINITION = "# A trajectory message that encodes joint limits within it.\ntrajectory_msgs/JointTrajectory trajectory\n\n# A vector of JointLimit messages.\n# Each message contains the limits for a specific joint\narm_navigation_msgs/JointLimits[] limits\n";
  trajectory_msgs.JointTrajectory getTrajectory();
  void setTrajectory(trajectory_msgs.JointTrajectory value);
  java.util.List<arm_navigation_msgs.JointLimits> getLimits();
  void setLimits(java.util.List<arm_navigation_msgs.JointLimits> value);
}
