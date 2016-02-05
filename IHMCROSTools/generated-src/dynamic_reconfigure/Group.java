package dynamic_reconfigure;

public interface Group extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/Group";
  static final java.lang.String _DEFINITION = "string name\nstring type\nParamDescription[] parameters\nint32 parent \nint32 id\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getType();
  void setType(java.lang.String value);
  java.util.List<dynamic_reconfigure.ParamDescription> getParameters();
  void setParameters(java.util.List<dynamic_reconfigure.ParamDescription> value);
  int getParent();
  void setParent(int value);
  int getId();
  void setId(int value);
}
