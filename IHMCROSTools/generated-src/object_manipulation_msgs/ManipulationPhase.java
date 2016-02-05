package object_manipulation_msgs;

public interface ManipulationPhase extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ManipulationPhase";
  static final java.lang.String _DEFINITION = "int32 CHECKING_FEASIBILITY = 0\nint32 MOVING_TO_PREGRASP = 1\nint32 MOVING_TO_GRASP = 2\nint32 CLOSING = 3 \nint32 ADJUSTING_GRASP = 4\nint32 LIFTING = 5\nint32 MOVING_WITH_OBJECT = 6\nint32 MOVING_TO_PLACE = 7\nint32 PLACING = 8\nint32 OPENING = 9\nint32 RETREATING = 10\nint32 MOVING_WITHOUT_OBJECT = 11\nint32 SHAKING = 12\nint32 SUCCEEDED = 13\nint32 FAILED = 14\nint32 ABORTED = 15\nint32 HOLDING_OBJECT = 16\n\nint32 phase";
  static final int CHECKING_FEASIBILITY = 0;
  static final int MOVING_TO_PREGRASP = 1;
  static final int MOVING_TO_GRASP = 2;
  static final int CLOSING = 3;
  static final int ADJUSTING_GRASP = 4;
  static final int LIFTING = 5;
  static final int MOVING_WITH_OBJECT = 6;
  static final int MOVING_TO_PLACE = 7;
  static final int PLACING = 8;
  static final int OPENING = 9;
  static final int RETREATING = 10;
  static final int MOVING_WITHOUT_OBJECT = 11;
  static final int SHAKING = 12;
  static final int SUCCEEDED = 13;
  static final int FAILED = 14;
  static final int ABORTED = 15;
  static final int HOLDING_OBJECT = 16;
  int getPhase();
  void setPhase(int value);
}
