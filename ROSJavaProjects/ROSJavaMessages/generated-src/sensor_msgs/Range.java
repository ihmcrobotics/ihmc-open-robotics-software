package sensor_msgs;

public interface Range extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/Range";
  static final java.lang.String _DEFINITION = "# Single range reading from an active ranger that emits energy and reports\n# one range reading that is valid along an arc at the distance measured. \n# This message is not appropriate for fixed-range obstacle detectors, \n# such as the Sharp GP2D15. This message is also not appropriate for laser \n# scanners. See the LaserScan message if you are working with a laser scanner.\n\nHeader header    \t# timestamp in the header is the time the ranger\n\t\t \t# returned the distance reading\n\n# Radiation type enums\n# If you want a value added to this list, send an email to the ros-users list\nuint8 ULTRASOUND=0\nuint8 INFRARED=1\n\nuint8 radiation_type    # the type of radiation used by the sensor\n\t\t \t# (sound, IR, etc) [enum]\n\nfloat32 field_of_view   # the size of the arc that the distance reading is\n\t\t \t# valid for [rad]\n\t\t \t# the object causing the range reading may have\n\t\t \t# been anywhere within -field_of_view/2 and\n\t\t \t# field_of_view/2 at the measured range. \n                        # 0 angle corresponds to the x-axis of the sensor.\n\nfloat32 min_range       # minimum range value [m]\nfloat32 max_range       # maximum range value [m]\n\nfloat32 range           # range data [m]\n\t\t \t# (Note: values < range_min or > range_max\n\t\t \t# should be discarded)\n";
  static final byte ULTRASOUND = 0;
  static final byte INFRARED = 1;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getRadiationType();
  void setRadiationType(byte value);
  float getFieldOfView();
  void setFieldOfView(float value);
  float getMinRange();
  void setMinRange(float value);
  float getMaxRange();
  void setMaxRange(float value);
  float getRange();
  void setRange(float value);
}
