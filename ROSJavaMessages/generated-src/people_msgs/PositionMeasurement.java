package people_msgs;

public interface PositionMeasurement extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "people_msgs/PositionMeasurement";
  static final java.lang.String _DEFINITION = "Header          header\nstring          name\nstring          object_id\ngeometry_msgs/Point pos\nfloat64         reliability\nfloat64[9] \tcovariance\nbyte            initialization";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getObjectId();
  void setObjectId(java.lang.String value);
  geometry_msgs.Point getPos();
  void setPos(geometry_msgs.Point value);
  double getReliability();
  void setReliability(double value);
  double[] getCovariance();
  void setCovariance(double[] value);
  byte getInitialization();
  void setInitialization(byte value);
}
