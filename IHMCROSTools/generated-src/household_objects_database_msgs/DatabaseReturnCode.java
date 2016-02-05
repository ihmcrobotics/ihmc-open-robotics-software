package household_objects_database_msgs;

public interface DatabaseReturnCode extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "household_objects_database_msgs/DatabaseReturnCode";
  static final java.lang.String _DEFINITION = "# return codes for database-related services\n\nint32 UNKNOWN_ERROR = 1\nint32 DATABASE_NOT_CONNECTED = 2\nint32 DATABASE_QUERY_ERROR = 3\nint32 SUCCESS = -1\n\nint32 code";
  static final int UNKNOWN_ERROR = 1;
  static final int DATABASE_NOT_CONNECTED = 2;
  static final int DATABASE_QUERY_ERROR = 3;
  static final int SUCCESS = -1;
  int getCode();
  void setCode(int value);
}
