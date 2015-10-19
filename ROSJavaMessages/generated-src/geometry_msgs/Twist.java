package geometry_msgs;

public interface Twist extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Twist";
  static final java.lang.String _DEFINITION = "# This expresses velocity in free space broken into its linear and angular parts.\nVector3  linear\nVector3  angular\n";
  geometry_msgs.Vector3 getLinear();
  void setLinear(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getAngular();
  void setAngular(geometry_msgs.Vector3 value);
}
