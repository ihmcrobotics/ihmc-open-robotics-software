package diagnostic_msgs;

public interface DiagnosticStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "diagnostic_msgs/DiagnosticStatus";
  static final java.lang.String _DEFINITION = "# This message holds the status of an individual component of the robot.\n# \n\n# Possible levels of operations\nbyte OK=0\nbyte WARN=1\nbyte ERROR=2\nbyte STALE=3\n\nbyte level # level of operation enumerated above \nstring name # a description of the test/component reporting\nstring message # a description of the status\nstring hardware_id # a hardware unique string\nKeyValue[] values # an array of values associated with the status\n\n";
  static final byte OK = 0;
  static final byte WARN = 1;
  static final byte ERROR = 2;
  static final byte STALE = 3;
  byte getLevel();
  void setLevel(byte value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  java.lang.String getHardwareId();
  void setHardwareId(java.lang.String value);
  java.util.List<diagnostic_msgs.KeyValue> getValues();
  void setValues(java.util.List<diagnostic_msgs.KeyValue> value);
}
