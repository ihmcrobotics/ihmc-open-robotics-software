package arm_navigation_msgs;

public interface AttachedCollisionObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/AttachedCollisionObject";
  static final java.lang.String _DEFINITION = "# The CollisionObject will be attached with a fixed joint to this link\n# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation \n# is set to REMOVE will remove all attached bodies attached to any object\nstring link_name\n\n#Reserved for indicating that all attached objects should be removed\nstring REMOVE_ALL_ATTACHED_OBJECTS = \"all\"\n\n#This contains the actual shapes and poses for the CollisionObject\n#to be attached to the link\n#If action is remove and no object.id is set, all objects\n#attached to the link indicated by link_name will be removed\nCollisionObject object\n\n# The set of links that the attached objects are allowed to touch\n# by default - the link_name is included by default\nstring[] touch_links\n";
  static final java.lang.String REMOVE_ALL_ATTACHED_OBJECTS = "\"all\"";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  arm_navigation_msgs.CollisionObject getObject();
  void setObject(arm_navigation_msgs.CollisionObject value);
  java.util.List<java.lang.String> getTouchLinks();
  void setTouchLinks(java.util.List<java.lang.String> value);
}
