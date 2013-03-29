package sensor_msgs;

public interface PointField extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sensor_msgs/PointField";
  static final java.lang.String _DEFINITION = "# This message holds the description of one point entry in the\n# PointCloud2 message format.\nuint8 INT8    = 1\nuint8 UINT8   = 2\nuint8 INT16   = 3\nuint8 UINT16  = 4\nuint8 INT32   = 5\nuint8 UINT32  = 6\nuint8 FLOAT32 = 7\nuint8 FLOAT64 = 8\n\nstring name      # Name of field\nuint32 offset    # Offset from start of point struct\nuint8  datatype  # Datatype enumeration, see above\nuint32 count     # How many elements in the field\n";
  static final byte INT8 = 1;
  static final byte UINT8 = 2;
  static final byte INT16 = 3;
  static final byte UINT16 = 4;
  static final byte INT32 = 5;
  static final byte UINT32 = 6;
  static final byte FLOAT32 = 7;
  static final byte FLOAT64 = 8;
  java.lang.String getName();
  void setName(java.lang.String value);
  int getOffset();
  void setOffset(int value);
  byte getDatatype();
  void setDatatype(byte value);
  int getCount();
  void setCount(int value);
}
