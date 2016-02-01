package dynamic_reconfigure;

public interface ConfigDescription extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/ConfigDescription";
  static final java.lang.String _DEFINITION = "Group[] groups\nConfig max\nConfig min\nConfig dflt\n";
  java.util.List<dynamic_reconfigure.Group> getGroups();
  void setGroups(java.util.List<dynamic_reconfigure.Group> value);
  dynamic_reconfigure.Config getMax();
  void setMax(dynamic_reconfigure.Config value);
  dynamic_reconfigure.Config getMin();
  void setMin(dynamic_reconfigure.Config value);
  dynamic_reconfigure.Config getDflt();
  void setDflt(dynamic_reconfigure.Config value);
}
