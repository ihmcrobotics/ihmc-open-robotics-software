package kinematics_msgs;

public interface GetPositionFK extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetPositionFK";
  static final java.lang.String _DEFINITION = "# A service definition for a standard forward kinematics service\n# The frame_id in the header message is the frame in which \n# the forward kinematics poses will be returned\nHeader header\n\n# A vector of link name for which forward kinematics must be computed\nstring[] fk_link_names\n\n# A robot state consisting of joint names and joint positions to be used for forward kinematics\narm_navigation_msgs/RobotState robot_state\n---\n# The resultant vector of PoseStamped messages that contains the (stamped) poses of the requested links\ngeometry_msgs/PoseStamped[] pose_stamped\n\n# The list of link names corresponding to the poses\nstring[] fk_link_names\n\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n";
}
