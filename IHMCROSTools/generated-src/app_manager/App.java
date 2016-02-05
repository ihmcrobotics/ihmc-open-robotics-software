package app_manager;

public interface App extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/App";
  static final java.lang.String _DEFINITION = "# app name\nstring name\n# user-friendly display name of application\nstring display_name\n# icon for showing app\nIcon icon\n# ordered list (by preference) of client applications to interact with this robot app.  \nClientApp[] client_apps\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDisplayName();
  void setDisplayName(java.lang.String value);
  app_manager.Icon getIcon();
  void setIcon(app_manager.Icon value);
  java.util.List<app_manager.ClientApp> getClientApps();
  void setClientApps(java.util.List<app_manager.ClientApp> value);
}
