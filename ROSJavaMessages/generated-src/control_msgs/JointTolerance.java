package control_msgs;

public interface JointTolerance extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/JointTolerance";
  static final java.lang.String _DEFINITION = "# The tolerances specify the amount the position, velocity, and\n# accelerations can vary from the setpoints.  For example, in the case\n# of trajectory control, when the actual position varies beyond\n# (desired position + position tolerance), the trajectory goal may\n# abort.\n# \n# There are two special values for tolerances:\n#  * 0 - The tolerance is unspecified and will remain at whatever the default is\n#  * -1 - The tolerance is \"erased\".  If there was a default, the joint will be\n#         allowed to move without restriction.\n\nstring name\nfloat64 position  # in radians or meters (for a revolute or prismatic joint, respectively)\nfloat64 velocity  # in rad/sec or m/sec\nfloat64 acceleration  # in rad/sec^2 or m/sec^2\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  double getPosition();
  void setPosition(double value);
  double getVelocity();
  void setVelocity(double value);
  double getAcceleration();
  void setAcceleration(double value);
}
