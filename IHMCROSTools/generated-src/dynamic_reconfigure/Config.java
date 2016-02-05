package dynamic_reconfigure;

public interface Config extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/Config";
  static final java.lang.String _DEFINITION = "BoolParameter[] bools\nIntParameter[] ints\nStrParameter[] strs\nDoubleParameter[] doubles\nGroupState[] groups\n";
  java.util.List<dynamic_reconfigure.BoolParameter> getBools();
  void setBools(java.util.List<dynamic_reconfigure.BoolParameter> value);
  java.util.List<dynamic_reconfigure.IntParameter> getInts();
  void setInts(java.util.List<dynamic_reconfigure.IntParameter> value);
  java.util.List<dynamic_reconfigure.StrParameter> getStrs();
  void setStrs(java.util.List<dynamic_reconfigure.StrParameter> value);
  java.util.List<dynamic_reconfigure.DoubleParameter> getDoubles();
  void setDoubles(java.util.List<dynamic_reconfigure.DoubleParameter> value);
  java.util.List<dynamic_reconfigure.GroupState> getGroups();
  void setGroups(java.util.List<dynamic_reconfigure.GroupState> value);
}
