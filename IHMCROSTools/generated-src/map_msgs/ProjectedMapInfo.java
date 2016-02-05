package map_msgs;

public interface ProjectedMapInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/ProjectedMapInfo";
  static final java.lang.String _DEFINITION = "string frame_id\nfloat64 x\nfloat64 y\nfloat64 width\nfloat64 height\nfloat64 min_z\nfloat64 max_z";
  java.lang.String getFrameId();
  void setFrameId(java.lang.String value);
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getWidth();
  void setWidth(double value);
  double getHeight();
  void setHeight(double value);
  double getMinZ();
  void setMinZ(double value);
  double getMaxZ();
  void setMaxZ(double value);
}
