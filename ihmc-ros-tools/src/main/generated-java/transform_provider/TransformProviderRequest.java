package transform_provider;

public interface TransformProviderRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "transform_provider/TransformProviderRequest";
  static final java.lang.String _DEFINITION = "time time\nstring src\nstring dest\n";
  org.ros.message.Time getTime();
  void setTime(org.ros.message.Time value);
  java.lang.String getSrc();
  void setSrc(java.lang.String value);
  java.lang.String getDest();
  void setDest(java.lang.String value);
}
