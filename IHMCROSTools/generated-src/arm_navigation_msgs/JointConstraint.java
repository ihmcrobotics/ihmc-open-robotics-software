package arm_navigation_msgs;

public interface JointConstraint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/JointConstraint";
  static final java.lang.String _DEFINITION = "# Constrain the position of a joint to be within a certain bound\nstring joint_name\n\n# the bound to be achieved is [position - tolerance_below, position + tolerance_above]\nfloat64 position\nfloat64 tolerance_above\nfloat64 tolerance_below\n\n# A weighting factor for this constraint\nfloat64 weight";
  java.lang.String getJointName();
  void setJointName(java.lang.String value);
  double getPosition();
  void setPosition(double value);
  double getToleranceAbove();
  void setToleranceAbove(double value);
  double getToleranceBelow();
  void setToleranceBelow(double value);
  double getWeight();
  void setWeight(double value);
}
