package chomp_motion_planner;

public interface GetChompCollisionCostResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/GetChompCollisionCostResponse";
  static final java.lang.String _DEFINITION = "\n# Returns an array of costs for the links (in the same order as the input)\nfloat64[] costs\n\n# Each element of this array is a num_joints x 1 array that represents the\n# joint velocity that will move the link down the gradient of the distance\n# field (away from collisions)\nJointVelocityArray[] gradient\n\n# A boolean value which indicates whether the configuration is in collision\nuint8 in_collision";
  double[] getCosts();
  void setCosts(double[] value);
  java.util.List<chomp_motion_planner.JointVelocityArray> getGradient();
  void setGradient(java.util.List<chomp_motion_planner.JointVelocityArray> value);
  byte getInCollision();
  void setInCollision(byte value);
}
