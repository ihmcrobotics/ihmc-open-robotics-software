package collider;

public interface OccupancyPointQueryResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyPointQueryResponse";
  static final java.lang.String _DEFINITION = "int8 occupancy\nint8 FREE=0\nint8 OCCUPIED=1\nint8 UNKNOWN=-1";
  static final byte FREE = 0;
  static final byte OCCUPIED = 1;
  static final byte UNKNOWN = -1;
  byte getOccupancy();
  void setOccupancy(byte value);
}
