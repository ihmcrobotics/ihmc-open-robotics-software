package object_manipulation_msgs;

public interface GraspResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspResult";
  static final java.lang.String _DEFINITION = "int32 SUCCESS = 1\nint32 GRASP_OUT_OF_REACH = 2\nint32 GRASP_IN_COLLISION = 3\nint32 GRASP_UNFEASIBLE = 4\nint32 PREGRASP_OUT_OF_REACH = 5\nint32 PREGRASP_IN_COLLISION = 6\nint32 PREGRASP_UNFEASIBLE = 7\nint32 LIFT_OUT_OF_REACH = 8\nint32 LIFT_IN_COLLISION = 9\nint32 LIFT_UNFEASIBLE = 10\nint32 MOVE_ARM_FAILED = 11\nint32 GRASP_FAILED = 12\nint32 LIFT_FAILED = 13\nint32 RETREAT_FAILED = 14\nint32 result_code\n\n# whether the state of the world was disturbed by this attempt. generally, this flag\n# shows if another task can be attempted, or a new sensed world model is recommeded\n# before proceeding\nbool continuation_possible\n";
  static final int SUCCESS = 1;
  static final int GRASP_OUT_OF_REACH = 2;
  static final int GRASP_IN_COLLISION = 3;
  static final int GRASP_UNFEASIBLE = 4;
  static final int PREGRASP_OUT_OF_REACH = 5;
  static final int PREGRASP_IN_COLLISION = 6;
  static final int PREGRASP_UNFEASIBLE = 7;
  static final int LIFT_OUT_OF_REACH = 8;
  static final int LIFT_IN_COLLISION = 9;
  static final int LIFT_UNFEASIBLE = 10;
  static final int MOVE_ARM_FAILED = 11;
  static final int GRASP_FAILED = 12;
  static final int LIFT_FAILED = 13;
  static final int RETREAT_FAILED = 14;
  int getResultCode();
  void setResultCode(int value);
  boolean getContinuationPossible();
  void setContinuationPossible(boolean value);
}
