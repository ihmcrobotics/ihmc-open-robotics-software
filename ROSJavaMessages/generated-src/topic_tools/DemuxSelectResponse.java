package topic_tools;

public interface DemuxSelectResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "topic_tools/DemuxSelectResponse";
  static final java.lang.String _DEFINITION = "string prev_topic";
  java.lang.String getPrevTopic();
  void setPrevTopic(java.lang.String value);
}
