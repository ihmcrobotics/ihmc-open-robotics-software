package test_ros;

public interface TestArrays extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/TestArrays";
  static final java.lang.String _DEFINITION = "# caller_id of most recent node to send this message\nstring caller_id\n# caller_id of the original node to send this message\nstring orig_caller_id\n\nint32[] int32_array\nfloat32[] float32_array\ntime[] time_array\nTestString[] test_string_array\n# TODO: array of arrays\n";
  java.lang.String getCallerId();
  void setCallerId(java.lang.String value);
  java.lang.String getOrigCallerId();
  void setOrigCallerId(java.lang.String value);
  int[] getInt32Array();
  void setInt32Array(int[] value);
  float[] getFloat32Array();
  void setFloat32Array(float[] value);
  java.util.List<org.ros.message.Time> getTimeArray();
  void setTimeArray(java.util.List<org.ros.message.Time> value);
  java.util.List<test_ros.TestString> getTestStringArray();
  void setTestStringArray(java.util.List<test_ros.TestString> value);
}
