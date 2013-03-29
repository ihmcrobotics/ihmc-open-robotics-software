package topic_tools;

public interface MuxListResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "topic_tools/MuxListResponse";
  static final java.lang.String _DEFINITION = "string[] topics";
  java.util.List<java.lang.String> getTopics();
  void setTopics(java.util.List<java.lang.String> value);
}
