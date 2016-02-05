package tf2_msgs;

public interface TF2Error extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf2_msgs/TF2Error";
  static final java.lang.String _DEFINITION = "uint8 NO_ERROR = 0\nuint8 LOOKUP_ERROR = 1\nuint8 CONNECTIVITY_ERROR = 2\nuint8 EXTRAPOLATION_ERROR = 3\nuint8 INVALID_ARGUMENT_ERROR = 4\nuint8 TIMEOUT_ERROR = 5\nuint8 TRANSFORM_ERROR = 6\n\nuint8 error\nstring error_string\n";
  static final byte NO_ERROR = 0;
  static final byte LOOKUP_ERROR = 1;
  static final byte CONNECTIVITY_ERROR = 2;
  static final byte EXTRAPOLATION_ERROR = 3;
  static final byte INVALID_ARGUMENT_ERROR = 4;
  static final byte TIMEOUT_ERROR = 5;
  static final byte TRANSFORM_ERROR = 6;
  byte getError();
  void setError(byte value);
  java.lang.String getErrorString();
  void setErrorString(java.lang.String value);
}
