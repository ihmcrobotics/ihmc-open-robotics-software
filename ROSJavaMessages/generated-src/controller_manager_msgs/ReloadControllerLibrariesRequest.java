package controller_manager_msgs;

public interface ReloadControllerLibrariesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "controller_manager_msgs/ReloadControllerLibrariesRequest";
  static final java.lang.String _DEFINITION = "# The ReloadControllerLibraries service will reload all controllers that are available in\n# the system as plugins\n\n\n# Reloading libraries only works if there are no controllers loaded. If there\n# are still some controllers loaded, the reloading will fail.\n# If this bool is set to true, all loaded controllers will get\n# killed automatically, and the reloading can succeed.\nbool force_kill\n";
  boolean getForceKill();
  void setForceKill(boolean value);
}
