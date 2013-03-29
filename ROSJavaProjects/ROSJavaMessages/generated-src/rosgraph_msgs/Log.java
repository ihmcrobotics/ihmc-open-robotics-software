package rosgraph_msgs;

public interface Log extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rosgraph_msgs/Log";
  static final java.lang.String _DEFINITION = "##\n## Severity level constants\n##\nbyte DEBUG=1 #debug level\nbyte INFO=2  #general level\nbyte WARN=4  #warning level\nbyte ERROR=8 #error level\nbyte FATAL=16 #fatal/critical level\n##\n## Fields\n##\nHeader header\nbyte level\nstring name # name of the node\nstring msg # message \nstring file # file the message came from\nstring function # function the message came from\nuint32 line # line the message came from\nstring[] topics # topic names that the node publishes\n";
  static final byte DEBUG = 1;
  static final byte INFO = 2;
  static final byte WARN = 4;
  static final byte ERROR = 8;
  static final byte FATAL = 16;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getLevel();
  void setLevel(byte value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
  java.lang.String getFile();
  void setFile(java.lang.String value);
  java.lang.String getFunction();
  void setFunction(java.lang.String value);
  int getLine();
  void setLine(int value);
  java.util.List<java.lang.String> getTopics();
  void setTopics(java.util.List<java.lang.String> value);
}
