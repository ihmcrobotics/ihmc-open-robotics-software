package handle_msgs;

public interface HandleControl extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "handle_msgs/HandleControl";
  static final java.lang.String _DEFINITION = "# Command to move the HANDLE hand\n# \n\n# The different control types\nuint8 VELOCITY = 1\nuint8 POSITION = 2\nuint8 CURRENT  = 3\nuint8 VOLTAGE  = 4\nuint8 ANGLE    = 5\n\n# The control type for each motor.\nint32[5] type\n\n# The value to set.\nint32[5] value\n\n# Whether or not to control each motor.\nbool[5] valid\n\n# To be added if/when this becomes a service call:\n#---\n#bool ok\n#string reason\n";
  static final byte VELOCITY = 1;
  static final byte POSITION = 2;
  static final byte CURRENT = 3;
  static final byte VOLTAGE = 4;
  static final byte ANGLE = 5;
  int[] getType();
  void setType(int[] value);
  int[] getValue();
  void setValue(int[] value);
  boolean[] getValid();
  void setValid(boolean[] value);
}
