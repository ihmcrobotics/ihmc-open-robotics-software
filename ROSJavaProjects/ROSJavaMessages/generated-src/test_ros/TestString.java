package test_ros;

public interface TestString extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/TestString";
  static final java.lang.String _DEFINITION = "# Integration test message\n# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\nstring data\n";
  java.lang.String getCallerId();
  void setCallerId(java.lang.String value);
  java.lang.String getOrigCallerId();
  void setOrigCallerId(java.lang.String value);
  java.lang.String getData();
  void setData(java.lang.String value);
}
