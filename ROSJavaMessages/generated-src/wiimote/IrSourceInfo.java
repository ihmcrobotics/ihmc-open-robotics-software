package wiimote;

public interface IrSourceInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "wiimote/IrSourceInfo";
  static final java.lang.String _DEFINITION = "# Sensor data pertaining to the Wiimote infrared camera.\n# This message contains data for one of the four infrared \n# light sources that the camera detects.\n#\n# Each light is specified with a 2D position and \n# a \'source magnitude\' (ir_size). If the x dimension\n# is set to INVALID_FLOAT, then no light was detected for \n# the respective light. The Wiimote handles up to\n# four light sources, and the wiimote_node.py software\n# is written to that limit as well.\n#\n# I am unsure what the \'ir_size\' values represent. \n# They are described as \'source magnitude\' in some places. I\n# *assume* this is signal amplitude, but it\'s unclear. \n# Note that current lowest level cwiid driver does not \n# seem to pass the ir_size value to the cwiid Wiimote.c. \n# For now this size will therefore be set INVALID\n\nfloat64 x \nfloat64 y \nint64 ir_size\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  long getIrSize();
  void setIrSize(long value);
}
