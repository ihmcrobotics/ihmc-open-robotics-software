package gazebo;

public interface SpawnModelRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SpawnModelRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                 # name of the model to be spawn\nstring model_xml                  # this should be an urdf or gazebo xml\nstring robot_namespace            # spawn robot and all ROS interfaces under this namespace\ngeometry_msgs/Pose initial_pose   # only applied to canonical body\nstring reference_frame            # initial_pose is defined relative to the frame of this model/body\n                                  # if left empty or \"world\", then gazebo world frame is used\n                                  # if non-existent model/body is specified, an error is returned\n                                  #   and the model is not spawned\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
  java.lang.String getModelXml();
  void setModelXml(java.lang.String value);
  java.lang.String getRobotNamespace();
  void setRobotNamespace(java.lang.String value);
  geometry_msgs.Pose getInitialPose();
  void setInitialPose(geometry_msgs.Pose value);
  java.lang.String getReferenceFrame();
  void setReferenceFrame(java.lang.String value);
}
