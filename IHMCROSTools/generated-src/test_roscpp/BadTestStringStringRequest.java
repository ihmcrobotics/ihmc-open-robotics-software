package test_roscpp;

public interface BadTestStringStringRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roscpp/BadTestStringStringRequest";
  static final java.lang.String _DEFINITION = "# field name is different, so md5 sum should be different\nstring strbad\n";
  java.lang.String getStrbad();
  void setStrbad(java.lang.String value);
}
