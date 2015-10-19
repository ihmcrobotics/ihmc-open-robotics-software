package atlas_msgs;

public interface AtlasPositionData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasPositionData";
  static final java.lang.String _DEFINITION = "# mirrors AtlasPositionData in AtlasControlTypes.h\ngeometry_msgs/Vector3 position\ngeometry_msgs/Vector3 velocity\n";
  geometry_msgs.Vector3 getPosition();
  void setPosition(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getVelocity();
  void setVelocity(geometry_msgs.Vector3 value);
}
