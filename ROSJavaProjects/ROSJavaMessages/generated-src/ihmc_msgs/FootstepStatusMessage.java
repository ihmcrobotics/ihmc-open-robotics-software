package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusMessage\r\n# This message gives the status of the current footstep from the controller as well as the position\r\n# and orientation of the footstep in world cooredinates. \r\n\r\n# Options for status\r\nuint8 STARTED=0 # execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state\r\nuint8 COMPLETED=1 # a footstep is completed\r\nuint8 status\r\n\r\n# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\r\n# FootstepDataListMessage.\r\nint32 footstep_index\r\n\r\n# Options for robotSide\r\nuint8 LEFT=0 # refers to the LEFT side of a robot\r\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\r\nuint8 robot_side\r\n\r\n# actualFootPositionInWorld gives the position of where the foot actually landed as opposed\r\n# to the desired position sent to the controller\r\ngeometry_msgs/Vector3 actual_foot_position_in_world\r\n\r\n# actualFootOrientationInWorld gives the orientation the foot is actually in as opposed tothe desired orientation sent to the controller\r\ngeometry_msgs/Quaternion actual_foot_orientation_in_world\r\n\r\n# isDoneWalking will be set to true when in double-support and there are no footsteps queued. If\r\n# isDoneWalking is set to true, the rest of the fields in this packet should be ignored\r\nbool is_done_walking\r\n\r\n\r\n";
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
  geometry_msgs.Vector3 getActualFootPositionInWorld();
  void setActualFootPositionInWorld(geometry_msgs.Vector3 value);
  geometry_msgs.Quaternion getActualFootOrientationInWorld();
  void setActualFootOrientationInWorld(geometry_msgs.Quaternion value);
  boolean getIsDoneWalking();
  void setIsDoneWalking(boolean value);
}
