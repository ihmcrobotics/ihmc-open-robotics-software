package people_msgs;

public interface Person extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "people_msgs/Person";
  static final java.lang.String _DEFINITION = "string              name\ngeometry_msgs/Point position\ngeometry_msgs/Point velocity\nfloat64             reliability\nstring[]            tagnames\nstring[]            tags\n\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  geometry_msgs.Point getVelocity();
  void setVelocity(geometry_msgs.Point value);
  double getReliability();
  void setReliability(double value);
  java.util.List<java.lang.String> getTagnames();
  void setTagnames(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getTags();
  void setTags(java.util.List<java.lang.String> value);
}
