package ihmc_msgs;

public interface FootstepStatusMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/FootstepStatusMessage";
  static final java.lang.String _DEFINITION = "## FootstepStatusMessage\n# This message gives the status of the current footstep from the controller as well as the position\n# and orientation of the footstep in world cooredinates. \n\n# Options for status\nuint8 STARTED=0 # execution of a footstep has begun. actualFootPositionInWorld and actualFootOrientationInWorld should be ignored in this state\nuint8 COMPLETED=1 # a footstep is completed\nuint8 status\n\n# footstepIndex starts at 0 and monotonically increases with each completed footstep in a given\n# FootstepDataListMessage.\nint32 footstep_index\n\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n# actualFootPositionInWorld gives the position of where the foot actually landed as opposed\n# to the desired position sent to the controller\ngeometry_msgs/Vector3 actual_foot_position_in_world\n\n# actualFootOrientationInWorld gives the orientation the foot is actually in as opposed tothe desired orientation sent to the controller\ngeometry_msgs/Quaternion actual_foot_orientation_in_world\n\n# isDoneWalking will be set to true when in double-support and there are no footsteps queued. If\n# isDoneWalking is set to true, the rest of the fields in this packet should be ignored\nbool is_done_walking\n\n\n";
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
