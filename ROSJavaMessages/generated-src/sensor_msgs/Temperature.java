package sensor_msgs;

public interface Temperature extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/Temperature";
  static final java.lang.String _DEFINITION = " # Single temperature reading.\n\n Header header           # timestamp is the time the temperature was measured\n                         # frame_id is the location of the temperature reading\n\n float64 temperature     # Measurement of the Temperature in Degrees Celsius\n\n float64 variance        # 0 is interpreted as variance unknown";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getTemperature();
  void setTemperature(double value);
  double getVariance();
  void setVariance(double value);
}
