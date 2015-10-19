package app_manager;

public interface GetAppDetailsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/GetAppDetailsRequest";
  static final java.lang.String _DEFINITION = "# Name of the app to get details of\nstring name \n";
  java.lang.String getName();
  void setName(java.lang.String value);
}
