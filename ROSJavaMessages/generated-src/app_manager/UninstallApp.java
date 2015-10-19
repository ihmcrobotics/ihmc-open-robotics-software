package app_manager;

public interface UninstallApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/UninstallApp";
  static final java.lang.String _DEFINITION = "# Name of app to uninstall\nstring name\n---\n# true if app stopped, false otherwise\nbool uninstalled\nstring message\n";
}
