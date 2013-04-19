package object_manipulation_msgs;

public interface GraspPlanningErrorCode extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspPlanningErrorCode";
  static final java.lang.String _DEFINITION = "# Error codes for grasp and place planning\n\n# plan completed as expected\nint32 SUCCESS = 0\n\n# tf error encountered while transforming\nint32 TF_ERROR = 1 \n\n# some other error\nint32 OTHER_ERROR = 2\n\n# the actual value of this error code\nint32 value";
  static final int SUCCESS = 0;
  static final int TF_ERROR = 1;
  static final int OTHER_ERROR = 2;
  int getValue();
  void setValue(int value);
}
