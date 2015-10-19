package arm_navigation_msgs;

public interface WorkspaceParameters extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/WorkspaceParameters";
  static final java.lang.String _DEFINITION = "# This message contains a set of parameters useful in\n# setting up the workspace for planning\narm_navigation_msgs/Shape  workspace_region_shape\ngeometry_msgs/PoseStamped    workspace_region_pose\n\n";
  arm_navigation_msgs.Shape getWorkspaceRegionShape();
  void setWorkspaceRegionShape(arm_navigation_msgs.Shape value);
  geometry_msgs.PoseStamped getWorkspaceRegionPose();
  void setWorkspaceRegionPose(geometry_msgs.PoseStamped value);
}
