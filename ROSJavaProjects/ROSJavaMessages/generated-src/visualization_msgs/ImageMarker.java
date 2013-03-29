package visualization_msgs;

public interface ImageMarker extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "visualization_msgs/ImageMarker";
  static final java.lang.String _DEFINITION = "uint8 CIRCLE=0\nuint8 LINE_STRIP=1\nuint8 LINE_LIST=2\nuint8 POLYGON=3\nuint8 POINTS=4\n\nuint8 ADD=0\nuint8 REMOVE=1\n\nHeader header\nstring ns\t\t# namespace, used with id to form a unique id\nint32 id          \t# unique id within the namespace\nint32 type        \t# CIRCLE/LINE_STRIP/etc.\nint32 action      \t# ADD/REMOVE\ngeometry_msgs/Point position # 2D, in pixel-coords\nfloat32 scale\t \t# the diameter for a circle, etc.\nstd_msgs/ColorRGBA outline_color\nuint8 filled\t\t# whether to fill in the shape with color\nstd_msgs/ColorRGBA fill_color # color [0.0-1.0]\nduration lifetime       # How long the object should last before being automatically deleted.  0 means forever\n\n\ngeometry_msgs/Point[] points # used for LINE_STRIP/LINE_LIST/POINTS/etc., 2D in pixel coords\nstd_msgs/ColorRGBA[] outline_colors # a color for each line, point, etc.";
  static final byte CIRCLE = 0;
  static final byte LINE_STRIP = 1;
  static final byte LINE_LIST = 2;
  static final byte POLYGON = 3;
  static final byte POINTS = 4;
  static final byte ADD = 0;
  static final byte REMOVE = 1;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getNs();
  void setNs(java.lang.String value);
  int getId();
  void setId(int value);
  int getType();
  void setType(int value);
  int getAction();
  void setAction(int value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  float getScale();
  void setScale(float value);
  std_msgs.ColorRGBA getOutlineColor();
  void setOutlineColor(std_msgs.ColorRGBA value);
  byte getFilled();
  void setFilled(byte value);
  std_msgs.ColorRGBA getFillColor();
  void setFillColor(std_msgs.ColorRGBA value);
  org.ros.message.Duration getLifetime();
  void setLifetime(org.ros.message.Duration value);
  java.util.List<geometry_msgs.Point> getPoints();
  void setPoints(java.util.List<geometry_msgs.Point> value);
  java.util.List<std_msgs.ColorRGBA> getOutlineColors();
  void setOutlineColors(java.util.List<std_msgs.ColorRGBA> value);
}
