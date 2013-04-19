package app_manager;

public interface StatusCodes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "app_manager/StatusCodes";
  static final java.lang.String _DEFINITION = "# Common error codes used with App Manager.\nint32 SUCCESS = 0\n# Request was invalid.\nint32 BAD_REQUEST = 400\n# App is not installed.\nint32 NOT_FOUND = 404\n# App is not running.\nint32 NOT_RUNNING = 430\n# Unknown internal error on the server.\nint32 INTERNAL_ERROR = 500\n# App is installed but failed validation.\nint32 APP_INVALID = 510\n# App manager does not support launching multiple apps simultaneously. Running app must first be stopped.\nint32 MULTIAPP_NOT_SUPPORTED = 511\n";
  static final int SUCCESS = 0;
  static final int BAD_REQUEST = 400;
  static final int NOT_FOUND = 404;
  static final int NOT_RUNNING = 430;
  static final int INTERNAL_ERROR = 500;
  static final int APP_INVALID = 510;
  static final int MULTIAPP_NOT_SUPPORTED = 511;
}
