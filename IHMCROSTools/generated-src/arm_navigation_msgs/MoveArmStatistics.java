package arm_navigation_msgs;

public interface MoveArmStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MoveArmStatistics";
  static final java.lang.String _DEFINITION = "int32 request_id\nstring result\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\nfloat64 planning_time\nfloat64 smoothing_time\nfloat64 ik_time\nfloat64 time_to_execution\nfloat64 time_to_result\n\nbool preempted\n\nfloat64 num_replans\nfloat64 trajectory_duration\n\nstring planner_service_name\n";
  int getRequestId();
  void setRequestId(int value);
  java.lang.String getResult();
  void setResult(java.lang.String value);
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
  double getPlanningTime();
  void setPlanningTime(double value);
  double getSmoothingTime();
  void setSmoothingTime(double value);
  double getIkTime();
  void setIkTime(double value);
  double getTimeToExecution();
  void setTimeToExecution(double value);
  double getTimeToResult();
  void setTimeToResult(double value);
  boolean getPreempted();
  void setPreempted(boolean value);
  double getNumReplans();
  void setNumReplans(double value);
  double getTrajectoryDuration();
  void setTrajectoryDuration(double value);
  java.lang.String getPlannerServiceName();
  void setPlannerServiceName(java.lang.String value);
}
