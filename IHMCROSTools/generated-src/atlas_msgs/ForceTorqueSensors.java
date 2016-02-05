package atlas_msgs;

public interface ForceTorqueSensors extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/ForceTorqueSensors";
  static final java.lang.String _DEFINITION = "# Atlas force torque sensors for the wrists and ankles\nHeader header\n\ngeometry_msgs/Wrench l_foot\ngeometry_msgs/Wrench r_foot\ngeometry_msgs/Wrench l_hand\ngeometry_msgs/Wrench r_hand\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Wrench getLFoot();
  void setLFoot(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getRFoot();
  void setRFoot(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getLHand();
  void setLHand(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getRHand();
  void setRHand(geometry_msgs.Wrench value);
}
