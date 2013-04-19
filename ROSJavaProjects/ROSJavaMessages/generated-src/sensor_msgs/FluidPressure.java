package sensor_msgs;

public interface FluidPressure extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/FluidPressure";
  static final java.lang.String _DEFINITION = " # Single pressure reading.  This message is appropriate for measuring the\n # pressure inside of a fluid (air, water, etc).  This also includes\n # atmospheric or barometric pressure.\n\n # This message is not appropriate for force/pressure contact sensors.\n\n Header header           # timestamp of the measurement\n                         # frame_id is the location of the pressure sensor\n\n float64 fluid_pressure  # Absolute pressure reading in Pascals.\n\n float64 variance        # 0 is interpreted as variance unknown";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getFluidPressure();
  void setFluidPressure(double value);
  double getVariance();
  void setVariance(double value);
}
