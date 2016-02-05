package topic_tools;

public interface MuxAddRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "topic_tools/MuxAddRequest";
  static final java.lang.String _DEFINITION = "string topic\n";
  java.lang.String getTopic();
  void setTopic(java.lang.String value);
}
