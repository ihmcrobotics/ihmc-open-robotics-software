package visualization_msgs;

public interface InteractiveMarkerInit extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/InteractiveMarkerInit";
  static final java.lang.String _DEFINITION = "# Identifying string. Must be unique in the topic namespace\n# that this server works on.\nstring server_id\n\n# Sequence number.\n# The client will use this to detect if it has missed a subsequent\n# update.  Every update message will have the same sequence number as\n# an init message.  Clients will likely want to unsubscribe from the\n# init topic after a successful initialization to avoid receiving\n# duplicate data.\nuint64 seq_num\n\n# All markers.\nInteractiveMarker[] markers\n";
  java.lang.String getServerId();
  void setServerId(java.lang.String value);
  long getSeqNum();
  void setSeqNum(long value);
  java.util.List<visualization_msgs.InteractiveMarker> getMarkers();
  void setMarkers(java.util.List<visualization_msgs.InteractiveMarker> value);
}
