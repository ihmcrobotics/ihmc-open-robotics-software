package arm_navigation_msgs;

public interface AllowedContactSpecification extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/AllowedContactSpecification";
  static final java.lang.String _DEFINITION = "# The names of the regions\nstring name\n\n# The shape of the region in the environment\narm_navigation_msgs/Shape shape\n\n# The pose of the space defining the region\ngeometry_msgs/PoseStamped pose_stamped\n\n# The set of links that will be allowed to have penetration contact within this region\nstring[] link_names\n\n# The maximum penetration depth allowed for every link\nfloat64 penetration_depth\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  arm_navigation_msgs.Shape getShape();
  void setShape(arm_navigation_msgs.Shape value);
  geometry_msgs.PoseStamped getPoseStamped();
  void setPoseStamped(geometry_msgs.PoseStamped value);
  java.util.List<java.lang.String> getLinkNames();
  void setLinkNames(java.util.List<java.lang.String> value);
  double getPenetrationDepth();
  void setPenetrationDepth(double value);
}
