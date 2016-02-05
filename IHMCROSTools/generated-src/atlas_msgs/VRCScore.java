package atlas_msgs;

public interface VRCScore extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/VRCScore";
  static final java.lang.String _DEFINITION = "# Potential values for task_type\nuint32 TASK_OTHER = 0\nuint32 TASK_DRIVING = 1\nuint32 TASK_WALKING = 2\nuint32 TASK_MANIPULATION = 3\n# Absolute wall time\ntime wall_time\n# Absolute sim time\ntime sim_time\n# Wall time elapsed since passing the first gate\ntime wall_time_elapsed\n# Sim time elapsed since passing the first gate\ntime sim_time_elapsed\n# How many parts of the task have been accomplished (e.g., gates traversed)\nint32 completion_score\n# How many damaging falls detected\nint32 falls\n# Optional message to describe events\nstring message\n# Which type of task\nuint32 task_type\n";
  static final int TASK_OTHER = 0;
  static final int TASK_DRIVING = 1;
  static final int TASK_WALKING = 2;
  static final int TASK_MANIPULATION = 3;
  org.ros.message.Time getWallTime();
  void setWallTime(org.ros.message.Time value);
  org.ros.message.Time getSimTime();
  void setSimTime(org.ros.message.Time value);
  org.ros.message.Time getWallTimeElapsed();
  void setWallTimeElapsed(org.ros.message.Time value);
  org.ros.message.Time getSimTimeElapsed();
  void setSimTimeElapsed(org.ros.message.Time value);
  int getCompletionScore();
  void setCompletionScore(int value);
  int getFalls();
  void setFalls(int value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  int getTaskType();
  void setTaskType(int value);
}
