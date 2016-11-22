package ihmc_msgs;

public interface FootstepStatusRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusRosMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusRosMessage\n# This message gives the status of the current footstep from the controller as well as the position\n# and orientation of the footstep in world cooredinates. \n\n# The current footstep status enum value.\nuint8 status\n\n# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\n# FootstepDataListMessage.\nint32 footstep_index\n\n# The robot side (left or right) that this footstep status correlates to.\nuint8 robot_side\n\n# desiredFootPositionInWorld gives the position of the desired position sent to the controller as\n# opposed to where the foot actually landed\ngeometry_msgs/Vector3 desired_foot_position_in_world\n\n# desiredFootOrientationInWorld gives the desired orientation of the foot sent to the controller as\n# opposed to the orientation where the foot actually is\ngeometry_msgs/Quaternion desired_foot_orientation_in_world\n\n# actualFootPositionInWorld gives the position of where the foot actually landed as opposed to the\n# desired position sent to the controller\ngeometry_msgs/Vector3 actual_foot_position_in_world\n\n# actualFootOrientationInWorld gives the orientation the foot is actually in as opposed to the desired\n# orientation sent to the controller\ngeometry_msgs/Quaternion actual_foot_orientation_in_world\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"status\" enum values:\nuint8 STARTED=0 # execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state\nuint8 COMPLETED=1 # a footstep is completed\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n";
  static final byte STARTED = 0;
  static final byte COMPLETED = 1;
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  byte getStatus();
  void setStatus(byte value);
  int getFootstepIndex();
  void setFootstepIndex(int value);
  byte getRobotSide();
  void setRobotSide(byte value);
  geometry_msgs.Vector3 getDesiredFootPositionInWorld();
  void setDesiredFootPositionInWorld(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getDesiredFootOrientationInWorld();
  void setDesiredFootOrientationInWorld(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getActualFootPositionInWorld();
  void setActualFootPositionInWorld(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getActualFootOrientationInWorld();
  void setActualFootOrientationInWorld(geometry_msgs.Quaternion value);
  long getUniqueId();
  void setUniqueId(long value);
}
