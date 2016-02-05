package actionlib_msgs;

public interface GoalStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "actionlib_msgs/GoalStatus";
  static final java.lang.String _DEFINITION = "GoalID goal_id\nuint8 status\nuint8 PENDING         = 0   # The goal has yet to be processed by the action server\nuint8 ACTIVE          = 1   # The goal is currently being processed by the action server\nuint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n                            #   and has since completed its execution (Terminal State)\nuint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\nuint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n                            #    to some failure (Terminal State)\nuint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n                            #    because the goal was unattainable or invalid (Terminal State)\nuint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n                            #    and has not yet completed execution\nuint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n                            #    but the action server has not yet confirmed that the goal is canceled\nuint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n                            #    and was successfully cancelled (Terminal State)\nuint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n                            #    sent over the wire by an action server\n\n#Allow for the user to associate a string with GoalStatus for debugging\nstring text\n\n";
  static final byte PENDING = 0;
  static final byte ACTIVE = 1;
  static final byte PREEMPTED = 2;
  static final byte SUCCEEDED = 3;
  static final byte ABORTED = 4;
  static final byte REJECTED = 5;
  static final byte PREEMPTING = 6;
  static final byte RECALLING = 7;
  static final byte RECALLED = 8;
  static final byte LOST = 9;
  actionlib_msgs.GoalID getGoalId();
  void setGoalId(actionlib_msgs.GoalID value);
  byte getStatus();
  void setStatus(byte value);
  java.lang.String getText();
  void setText(java.lang.String value);
}
