package ihmc_msgs;

public interface Point2dRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/Point2dRosMessage";
  static final java.lang.String _DEFINITION = "## Point2dRosMessage\n# This message represents a point on a 2d plane. The coordinates are referred to as \"x\" and \"y\"\n# The first coordinate of the point in a 2D plane\nfloat64 x\n\n# The second coordinate of the point in a 2D plane\nfloat64 y\n\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
}
