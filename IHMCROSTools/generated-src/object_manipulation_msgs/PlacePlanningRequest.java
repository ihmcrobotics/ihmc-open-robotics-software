package object_manipulation_msgs;

public interface PlacePlanningRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/PlacePlanningRequest";
  static final java.lang.String _DEFINITION = "# Requests that a list of possible place locations be generated for a given object\n\n# the arm being used\nstring arm_name\n\n# the object to be placed\nGraspableObject target\n\n# a default orientation for the object when placed\n# this is somewhat specific to the interactive manipulation tool gripper_click\n# and should be removed in the future\ngeometry_msgs/Quaternion default_orientation\n\n# The position of the end-effector for the grasp relative to the object\ngeometry_msgs/Pose grasp_pose\n\n# the name that the target object has in the collision environment\n# can be left empty if no name is available\nstring collision_object_name\n\n# the name that the support surface (e.g. table) has in the collision map\n# can be left empty if no name is available\nstring collision_support_surface_name\n\n";
  java.lang.String getArmName();
  void setArmName(java.lang.String value);
  object_manipulation_msgs.GraspableObject getTarget();
  void setTarget(object_manipulation_msgs.GraspableObject value);
  geometry_msgs.Quaternion getDefaultOrientation();
  void setDefaultOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Pose getGraspPose();
  void setGraspPose(geometry_msgs.Pose value);
  java.lang.String getCollisionObjectName();
  void setCollisionObjectName(java.lang.String value);
  java.lang.String getCollisionSupportSurfaceName();
  void setCollisionSupportSurfaceName(java.lang.String value);
}
