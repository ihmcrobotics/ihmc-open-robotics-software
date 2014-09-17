package ethz_asl_icp_mapper;

public interface GetModeResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/GetModeResponse";
  static final java.lang.String _DEFINITION = "bool localize\nbool map";
  boolean getLocalize();
  void setLocalize(boolean value);
  boolean getMap();
  void setMap(boolean value);
}
