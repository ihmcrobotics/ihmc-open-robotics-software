package arm_navigation_msgs;

public interface CollisionObjectOperation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/CollisionObjectOperation";
  static final java.lang.String _DEFINITION = "#Puts the object into the environment\n#or updates the object if already added\nbyte ADD=0\n\n#Removes the object from the environment entirely\nbyte REMOVE=1\n\n#Only valid within the context of a CollisionAttachedObject message\n#Will be ignored if sent with an CollisionObject message\n#Takes an attached object, detaches from the attached link\n#But adds back in as regular object\nbyte DETACH_AND_ADD_AS_OBJECT=2\n\n#Only valid within the context of a CollisionAttachedObject message\n#Will be ignored if sent with an CollisionObject message\n#Takes current object in the environment and removes it as\n#a regular object\nbyte ATTACH_AND_REMOVE_AS_OBJECT=3\n\n# Byte code for operation\nbyte operation\n";
  static final byte ADD = 0;
  static final byte REMOVE = 1;
  static final byte DETACH_AND_ADD_AS_OBJECT = 2;
  static final byte ATTACH_AND_REMOVE_AS_OBJECT = 3;
  byte getOperation();
  void setOperation(byte value);
}
