package test_roslib_comm;

public interface ArrayOfMsgs extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/ArrayOfMsgs";
  static final java.lang.String _DEFINITION = "std_msgs/String[] strings\nstd_msgs/Time[] times\nstd_msgs/MultiArrayLayout[] muls";
  java.util.List<std_msgs.String> getStrings();
  void setStrings(java.util.List<std_msgs.String> value);
  java.util.List<std_msgs.Time> getTimes();
  void setTimes(java.util.List<std_msgs.Time> value);
  java.util.List<std_msgs.MultiArrayLayout> getMuls();
  void setMuls(java.util.List<std_msgs.MultiArrayLayout> value);
}
