package sensor_msgs;

public interface RelativeHumidity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/RelativeHumidity";
  static final java.lang.String _DEFINITION = " # Single reading from a relative humidity sensor.  Defines the ratio of partial\n # pressure of water vapor to the saturated vapor pressure at a temperature.\n\n Header header             # timestamp of the measurement\n                           # frame_id is the location of the humidity sensor\n\n float64 relative_humidity # Expression of the relative humidity\n                           # from 0.0 to 1.0.\n                           # 0.0 is no partial pressure of water vapor\n                           # 1.0 represents partial pressure of saturation\n\n float64 variance          # 0 is interpreted as variance unknown";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getRelativeHumidity();
  void setRelativeHumidity(double value);
  double getVariance();
  void setVariance(double value);
}
