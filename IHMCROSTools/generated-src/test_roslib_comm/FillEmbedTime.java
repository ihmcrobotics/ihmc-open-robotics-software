package test_roslib_comm;

public interface FillEmbedTime extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_roslib_comm/FillEmbedTime";
  static final java.lang.String _DEFINITION = "time t\nduration d\nstd_msgs/String str_msg\nstd_msgs/String[] str_msg_array\nint32 i32";
  org.ros.message.Time getT();
  void setT(org.ros.message.Time value);
  org.ros.message.Duration getD();
  void setD(org.ros.message.Duration value);
  std_msgs.String getStrMsg();
  void setStrMsg(std_msgs.String value);
  java.util.List<std_msgs.String> getStrMsgArray();
  void setStrMsgArray(java.util.List<std_msgs.String> value);
  int getI32();
  void setI32(int value);
}
