package app_manager;

public interface GetInstallationStateResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/GetInstallationStateResponse";
  static final java.lang.String _DEFINITION = "ExchangeApp[] installed_apps\nExchangeApp[] available_apps";
  java.util.List<app_manager.ExchangeApp> getInstalledApps();
  void setInstalledApps(java.util.List<app_manager.ExchangeApp> value);
  java.util.List<app_manager.ExchangeApp> getAvailableApps();
  void setAvailableApps(java.util.List<app_manager.ExchangeApp> value);
}
