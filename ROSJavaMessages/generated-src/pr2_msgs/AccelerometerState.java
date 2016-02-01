package pr2_msgs;

public interface AccelerometerState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/AccelerometerState";
  static final java.lang.String _DEFINITION = "#This captures acceleration measurements from the 3-dof accelerometer in the hand of the PR2\n#Units are meters / second / second\n#Vectors should be <X, Y, Z> in the frame of the gripper\n\n#The communication with the accelerometer is at approximately 3khz, but there is only good timestamping every 1ms\n#This means the samples should be interpreted as all having come from the 1 ms before the time in the header\n\nHeader header\ngeometry_msgs/Vector3[] samples\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<geometry_msgs.Vector3> getSamples();
  void setSamples(java.util.List<geometry_msgs.Vector3> value);
}
