package rosgraph_msgs;

public interface TopicStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rosgraph_msgs/TopicStatistics";
  static final java.lang.String _DEFINITION = "# name of the topic\nstring topic\n\n# node id of the publisher\nstring node_pub\n\n# node id of the subscriber\nstring node_sub\n\n# the statistics apply to this time window\ntime window_start\ntime window_stop\n\n# number of messages delivered during the window\nint32 delivered_msgs\n# numbers of messages dropped during the window\nint32 dropped_msgs\n\n# traffic during the window, in bytes\nint32 traffic\n\n# mean/stddev/max period between two messages\nduration period_mean\nduration period_stddev\nduration period_max\n\n# mean/stddev/max age of the message based on the\n# timestamp in the message header. In case the\n# message does not have a header, it will be 0.\nduration stamp_age_mean\nduration stamp_age_stddev\nduration stamp_age_max\n";
  java.lang.String getTopic();
  void setTopic(java.lang.String value);
  java.lang.String getNodePub();
  void setNodePub(java.lang.String value);
  java.lang.String getNodeSub();
  void setNodeSub(java.lang.String value);
  org.ros.message.Time getWindowStart();
  void setWindowStart(org.ros.message.Time value);
  org.ros.message.Time getWindowStop();
  void setWindowStop(org.ros.message.Time value);
  int getDeliveredMsgs();
  void setDeliveredMsgs(int value);
  int getDroppedMsgs();
  void setDroppedMsgs(int value);
  int getTraffic();
  void setTraffic(int value);
  org.ros.message.Duration getPeriodMean();
  void setPeriodMean(org.ros.message.Duration value);
  org.ros.message.Duration getPeriodStddev();
  void setPeriodStddev(org.ros.message.Duration value);
  org.ros.message.Duration getPeriodMax();
  void setPeriodMax(org.ros.message.Duration value);
  org.ros.message.Duration getStampAgeMean();
  void setStampAgeMean(org.ros.message.Duration value);
  org.ros.message.Duration getStampAgeStddev();
  void setStampAgeStddev(org.ros.message.Duration value);
  org.ros.message.Duration getStampAgeMax();
  void setStampAgeMax(org.ros.message.Duration value);
}
