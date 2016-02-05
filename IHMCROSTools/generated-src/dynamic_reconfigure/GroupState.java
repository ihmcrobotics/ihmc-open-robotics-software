package dynamic_reconfigure;

public interface GroupState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/GroupState";
  static final java.lang.String _DEFINITION = "string name\nbool state\nint32 id\nint32 parent\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  boolean getState();
  void setState(boolean value);
  int getId();
  void setId(int value);
  int getParent();
  void setParent(int value);
}
