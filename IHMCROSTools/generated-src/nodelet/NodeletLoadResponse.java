package nodelet;

public interface NodeletLoadResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nodelet/NodeletLoadResponse";
  static final java.lang.String _DEFINITION = "bool success";
  boolean getSuccess();
  void setSuccess(boolean value);
}
