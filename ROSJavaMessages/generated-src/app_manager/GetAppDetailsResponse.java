package app_manager;

public interface GetAppDetailsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/GetAppDetailsResponse";
  static final java.lang.String _DEFINITION = "ExchangeApp app";
  app_manager.ExchangeApp getApp();
  void setApp(app_manager.ExchangeApp value);
}
