package ihmc_msgs;

public interface Point2dRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/Point2dRosMessage";
  static final java.lang.String _DEFINITION = "## Point2dRosMessage\n# No Documentation Annotation Found\n\nfloat64 x\n\nfloat64 y\n\n\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
}
