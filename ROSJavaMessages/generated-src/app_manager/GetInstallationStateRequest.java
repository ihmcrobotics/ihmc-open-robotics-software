package app_manager;

public interface GetInstallationStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/GetInstallationStateRequest";
  static final java.lang.String _DEFINITION = "bool remote_update\n";
  boolean getRemoteUpdate();
  void setRemoteUpdate(boolean value);
}
