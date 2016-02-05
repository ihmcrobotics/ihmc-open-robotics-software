package object_manipulation_msgs;

public interface PlaceLocationResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/PlaceLocationResult";
  static final java.lang.String _DEFINITION = "int32 SUCCESS = 1\nint32 PLACE_OUT_OF_REACH = 2\nint32 PLACE_IN_COLLISION = 3\nint32 PLACE_UNFEASIBLE = 4\nint32 PREPLACE_OUT_OF_REACH = 5\nint32 PREPLACE_IN_COLLISION = 6\nint32 PREPLACE_UNFEASIBLE = 7\nint32 RETREAT_OUT_OF_REACH = 8\nint32 RETREAT_IN_COLLISION = 9\nint32 RETREAT_UNFEASIBLE = 10\nint32 MOVE_ARM_FAILED = 11\nint32 PLACE_FAILED = 12\nint32 RETREAT_FAILED = 13\nint32 result_code\n\n# whether the state of the world was disturbed by this attempt. generally, this flag\n# shows if another task can be attempted, or a new sensed world model is recommeded\n# before proceeding\nbool continuation_possible\n";
  static final int SUCCESS = 1;
  static final int PLACE_OUT_OF_REACH = 2;
  static final int PLACE_IN_COLLISION = 3;
  static final int PLACE_UNFEASIBLE = 4;
  static final int PREPLACE_OUT_OF_REACH = 5;
  static final int PREPLACE_IN_COLLISION = 6;
  static final int PREPLACE_UNFEASIBLE = 7;
  static final int RETREAT_OUT_OF_REACH = 8;
  static final int RETREAT_IN_COLLISION = 9;
  static final int RETREAT_UNFEASIBLE = 10;
  static final int MOVE_ARM_FAILED = 11;
  static final int PLACE_FAILED = 12;
  static final int RETREAT_FAILED = 13;
  int getResultCode();
  void setResultCode(int value);
  boolean getContinuationPossible();
  void setContinuationPossible(boolean value);
}
