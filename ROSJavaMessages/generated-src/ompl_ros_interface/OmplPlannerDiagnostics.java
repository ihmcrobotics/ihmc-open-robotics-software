package ompl_ros_interface;

public interface OmplPlannerDiagnostics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ompl_ros_interface/OmplPlannerDiagnostics";
  static final java.lang.String _DEFINITION = "string summary\nstring group\nstring planner\nstring result\nfloat64 planning_time\nint32 trajectory_size\nfloat64 trajectory_duration\nint32 state_allocator_size\n";
  java.lang.String getSummary();
  void setSummary(java.lang.String value);
  java.lang.String getGroup();
  void setGroup(java.lang.String value);
  java.lang.String getPlanner();
  void setPlanner(java.lang.String value);
  java.lang.String getResult();
  void setResult(java.lang.String value);
  double getPlanningTime();
  void setPlanningTime(double value);
  int getTrajectorySize();
  void setTrajectorySize(int value);
  double getTrajectoryDuration();
  void setTrajectoryDuration(double value);
  int getStateAllocatorSize();
  void setStateAllocatorSize(int value);
}
