package atlas_msgs;

public interface VRCScore extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/VRCScore";
  static final java.lang.String _DEFINITION = "# Absolute wall time\ntime wall_time\n# Absolute sim time\ntime sim_time\n# Wall time elapsed since passing the first gate\ntime wall_time_elapsed\n# Sim time elapsed since passing the first gate\ntime sim_time_elapsed\n# How many parts of the task have been accomplished (e.g., gates traversed)\nint32 completion_score\n# How many damaging falls detected\nint32 falls\n# Optional message to describe events\nstring message\n";
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
}
