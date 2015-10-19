package pr2_msgs;

public interface LaserScannerSignal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/LaserScannerSignal";
  static final java.lang.String _DEFINITION = "# This message is emitted by the laser tilt controller when the laser hits\n# one limit of its profile.\nHeader header\n\n# signal == 0 => Half profile complete\n# signal == 1 => Full Profile Complete\nint32 signal\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getSignal();
  void setSignal(int value);
}
