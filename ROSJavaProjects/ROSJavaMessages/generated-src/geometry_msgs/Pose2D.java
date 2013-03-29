package geometry_msgs;

public interface Pose2D extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Pose2D";
  static final java.lang.String _DEFINITION = "# This expresses a position and orientation on a 2D manifold.\n\nfloat64 x\nfloat64 y\nfloat64 theta";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getTheta();
  void setTheta(double value);
}
