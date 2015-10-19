package app_manager;

public interface ClientApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/ClientApp";
  static final java.lang.String _DEFINITION = "# like \"android\" or \"web\" or \"linux\"\nstring client_type\n\n# like \"intent = ros.android.teleop\" and \"accelerometer = true\", used to choose which ClientApp to use\nKeyValue[] manager_data\n\n# parameters which just get passed through to the client app.\nKeyValue[] app_data\n";
  java.lang.String getClientType();
  void setClientType(java.lang.String value);
  java.util.List<app_manager.KeyValue> getManagerData();
  void setManagerData(java.util.List<app_manager.KeyValue> value);
  java.util.List<app_manager.KeyValue> getAppData();
  void setAppData(java.util.List<app_manager.KeyValue> value);
}
