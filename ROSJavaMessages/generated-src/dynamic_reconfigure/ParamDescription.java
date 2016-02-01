package dynamic_reconfigure;

public interface ParamDescription extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/ParamDescription";
  static final java.lang.String _DEFINITION = "string name\nstring type\nuint32 level\nstring description\nstring edit_method\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getType();
  void setType(java.lang.String value);
  int getLevel();
  void setLevel(int value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  java.lang.String getEditMethod();
  void setEditMethod(java.lang.String value);
}
