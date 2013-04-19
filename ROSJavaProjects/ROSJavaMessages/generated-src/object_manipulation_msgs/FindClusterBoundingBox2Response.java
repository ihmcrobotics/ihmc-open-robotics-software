package object_manipulation_msgs;

public interface FindClusterBoundingBox2Response extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/FindClusterBoundingBox2Response";
  static final java.lang.String _DEFINITION = "\n#the pose of the box frame\ngeometry_msgs/PoseStamped pose\n\n#the dimensions of the box\ngeometry_msgs/Vector3 box_dims\n\n#whether there was an error\nint32 SUCCESS=0\nint32 TF_ERROR=1\nint32 error_code";
  static final int SUCCESS = 0;
  static final int TF_ERROR = 1;
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
  geometry_msgs.Vector3 getBoxDims();
  void setBoxDims(geometry_msgs.Vector3 value);
  int getErrorCode();
  void setErrorCode(int value);
}
