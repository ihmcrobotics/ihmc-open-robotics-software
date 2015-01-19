package topic_tools;

public interface DemuxListResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "topic_tools/DemuxListResponse";
  static final java.lang.String _DEFINITION = "string[] topics";
  java.util.List<java.lang.String> getTopics();
  void setTopics(java.util.List<java.lang.String> value);
}
