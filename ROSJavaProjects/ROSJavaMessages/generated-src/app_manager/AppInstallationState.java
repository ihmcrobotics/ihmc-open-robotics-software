package app_manager;

public interface AppInstallationState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/AppInstallationState";
  static final java.lang.String _DEFINITION = "ExchangeApp[] installed_apps\nExchangeApp[] available_apps\n";
  java.util.List<app_manager.ExchangeApp> getInstalledApps();
  void setInstalledApps(java.util.List<app_manager.ExchangeApp> value);
  java.util.List<app_manager.ExchangeApp> getAvailableApps();
  void setAvailableApps(java.util.List<app_manager.ExchangeApp> value);
}
