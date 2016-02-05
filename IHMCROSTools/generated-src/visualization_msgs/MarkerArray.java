package visualization_msgs;

public interface MarkerArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/MarkerArray";
  static final java.lang.String _DEFINITION = "Marker[] markers\n";
  java.util.List<visualization_msgs.Marker> getMarkers();
  void setMarkers(java.util.List<visualization_msgs.Marker> value);
}
