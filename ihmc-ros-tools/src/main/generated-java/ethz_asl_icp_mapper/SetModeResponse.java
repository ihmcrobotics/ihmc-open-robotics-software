package ethz_asl_icp_mapper;

public interface SetModeResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/SetModeResponse";
  static final java.lang.String _DEFINITION = "bool localize\nbool map";
  boolean getLocalize();
  void setLocalize(boolean value);
  boolean getMap();
  void setMap(boolean value);
}
