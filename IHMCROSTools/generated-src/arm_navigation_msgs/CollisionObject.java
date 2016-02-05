package arm_navigation_msgs;

public interface CollisionObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/CollisionObject";
  static final java.lang.String _DEFINITION = "# a header, used for interpreting the poses\nHeader header\n\n# the id of the object\nstring id\n\n# The padding used for filtering points near the object.\n# This does not affect collision checking for the object.  \n# Set to negative to get zero padding.\nfloat32 padding\n\n#This contains what is to be done with the object\nCollisionObjectOperation operation\n\n#the shapes associated with the object\narm_navigation_msgs/Shape[] shapes\n\n#the poses associated with the shapes - will be transformed using the header\ngeometry_msgs/Pose[] poses\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getId();
  void setId(java.lang.String value);
  float getPadding();
  void setPadding(float value);
  arm_navigation_msgs.CollisionObjectOperation getOperation();
  void setOperation(arm_navigation_msgs.CollisionObjectOperation value);
  java.util.List<arm_navigation_msgs.Shape> getShapes();
  void setShapes(java.util.List<arm_navigation_msgs.Shape> value);
  java.util.List<geometry_msgs.Pose> getPoses();
  void setPoses(java.util.List<geometry_msgs.Pose> value);
}
