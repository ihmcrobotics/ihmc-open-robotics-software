package nodelet;

public interface NodeletListResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nodelet/NodeletListResponse";
  static final java.lang.String _DEFINITION = "string[] nodelets";
  java.util.List<java.lang.String> getNodelets();
  void setNodelets(java.util.List<java.lang.String> value);
}
