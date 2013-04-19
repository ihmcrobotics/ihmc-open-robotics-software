package arm_navigation_msgs;

public interface Constraints extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/Constraints";
  static final java.lang.String _DEFINITION = "# This message contains a list of motion planning constraints.\n\narm_navigation_msgs/JointConstraint[] joint_constraints\narm_navigation_msgs/PositionConstraint[] position_constraints\narm_navigation_msgs/OrientationConstraint[] orientation_constraints\narm_navigation_msgs/VisibilityConstraint[] visibility_constraints\n";
  java.util.List<arm_navigation_msgs.JointConstraint> getJointConstraints();
  void setJointConstraints(java.util.List<arm_navigation_msgs.JointConstraint> value);
  java.util.List<arm_navigation_msgs.PositionConstraint> getPositionConstraints();
  void setPositionConstraints(java.util.List<arm_navigation_msgs.PositionConstraint> value);
  java.util.List<arm_navigation_msgs.OrientationConstraint> getOrientationConstraints();
  void setOrientationConstraints(java.util.List<arm_navigation_msgs.OrientationConstraint> value);
  java.util.List<arm_navigation_msgs.VisibilityConstraint> getVisibilityConstraints();
  void setVisibilityConstraints(java.util.List<arm_navigation_msgs.VisibilityConstraint> value);
}
