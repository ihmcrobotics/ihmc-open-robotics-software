package sensor_msgs;

public interface LaserEcho extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/LaserEcho";
  static final java.lang.String _DEFINITION = "# This message is a submessage of MultiEchoLaserScan and is not intended\n# to be used separately.\n\nfloat32[] echoes  # Multiple values of ranges or intensities.\n                  # Each array represents data from the same angle increment.";
  float[] getEchoes();
  void setEchoes(float[] value);
}
