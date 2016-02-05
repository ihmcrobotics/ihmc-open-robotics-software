package base_local_planner;

public interface Position2DInt extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "base_local_planner/Position2DInt";
  static final java.lang.String _DEFINITION = "int64 x\nint64 y";
  long getX();
  void setX(long value);
  long getY();
  void setY(long value);
}
