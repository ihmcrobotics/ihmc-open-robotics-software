package dynamic_reconfigure;

public interface ReconfigureResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "dynamic_reconfigure/ReconfigureResponse";
  static final java.lang.String _DEFINITION = "Config config";
  dynamic_reconfigure.Config getConfig();
  void setConfig(dynamic_reconfigure.Config value);
}
