package actionlib_msgs;

public interface GoalID extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "actionlib_msgs/GoalID";
  static final java.lang.String _DEFINITION = "# The stamp should store the time at which this goal was requested.\n# It is used by an action server when it tries to preempt all\n# goals that were requested before a certain time\ntime stamp\n\n# The id provides a way to associate feedback and\n# result message with specific goal requests. The id\n# specified must be unique.\nstring id\n\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  java.lang.String getId();
  void setId(java.lang.String value);
}
