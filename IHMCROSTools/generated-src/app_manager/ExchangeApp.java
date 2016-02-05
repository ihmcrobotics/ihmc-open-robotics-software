package app_manager;

public interface ExchangeApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/ExchangeApp";
  static final java.lang.String _DEFINITION = "# app name\nstring name\n# user-friendly display name of application\nstring display_name\n# the version of the package currently installed\nstring version\n# latest version of the package avaliable\nstring latest_version\n# the detailed description of the app\nstring description\n# icon for showing app\nIcon icon\n# hidden apps are not show - used for cases where multiple apps are in a deb\nbool hidden";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDisplayName();
  void setDisplayName(java.lang.String value);
  java.lang.String getVersion();
  void setVersion(java.lang.String value);
  java.lang.String getLatestVersion();
  void setLatestVersion(java.lang.String value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  app_manager.Icon getIcon();
  void setIcon(app_manager.Icon value);
  boolean getHidden();
  void setHidden(boolean value);
}
