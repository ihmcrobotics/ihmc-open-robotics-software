package geometry_msgs;

public interface Wrench extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Wrench";
  static final java.lang.String _DEFINITION = "# This represents force in free space, separated into\n# its linear and angular parts.\nVector3  force\nVector3  torque\n";
  geometry_msgs.Vector3 getForce();
  void setForce(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getTorque();
  void setTorque(geometry_msgs.Vector3 value);
}
