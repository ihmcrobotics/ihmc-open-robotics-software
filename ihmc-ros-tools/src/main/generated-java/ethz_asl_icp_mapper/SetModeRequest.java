package ethz_asl_icp_mapper;

public interface SetModeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/SetModeRequest";
  static final java.lang.String _DEFINITION = "bool localize\nbool map\nbool applyChange\n";
  boolean getLocalize();
  void setLocalize(boolean value);
  boolean getMap();
  void setMap(boolean value);
  boolean getApplyChange();
  void setApplyChange(boolean value);
}
