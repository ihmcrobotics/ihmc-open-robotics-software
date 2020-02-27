package foothold_adaptation_msg;

public interface Foothold extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "foothold_adaptation_msg/Foothold";
  static final java.lang.String _DEFINITION = "# Header (time and frame)\r\nHeader header\r\n\r\n# Foot type identifier (left, right etc.)\r\nstd_msgs/String type\r\n\r\n# Footstep position and orientation\r\ngeometry_msgs/Pose pose\r\n\r\n# Status flag (0: unknown, 1: verified, 2: do not change)\r\nint8 flag";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  std_msgs.String getType();
  void setType(std_msgs.String value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  byte getFlag();
  void setFlag(byte value);
}
