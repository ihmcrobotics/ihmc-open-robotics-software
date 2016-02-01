package people_msgs;

public interface PositionMeasurementArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "people_msgs/PositionMeasurementArray";
  static final java.lang.String _DEFINITION = "Header          header\n\n# All of the people found\npeople_msgs/PositionMeasurement[] people\n\n# The co-occurrence matrix between people\nfloat32[] cooccurrence";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<people_msgs.PositionMeasurement> getPeople();
  void setPeople(java.util.List<people_msgs.PositionMeasurement> value);
  float[] getCooccurrence();
  void setCooccurrence(float[] value);
}
