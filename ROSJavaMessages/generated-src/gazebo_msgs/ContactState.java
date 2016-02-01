package gazebo_msgs;

public interface ContactState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/ContactState";
  static final java.lang.String _DEFINITION = "string info                                   # text info on this contact\nstring collision1_name                        # name of contact collision1\nstring collision2_name                        # name of contact collision2\ngeometry_msgs/Wrench[] wrenches               # list of forces/torques\ngeometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF\ngeometry_msgs/Vector3[] contact_positions     # list of contact position\ngeometry_msgs/Vector3[] contact_normals       # list of contact normals\nfloat64[] depths                              # list of penetration depths\n";
  java.lang.String getInfo();
  void setInfo(java.lang.String value);
  java.lang.String getCollision1Name();
  void setCollision1Name(java.lang.String value);
  java.lang.String getCollision2Name();
  void setCollision2Name(java.lang.String value);
  java.util.List<geometry_msgs.Wrench> getWrenches();
  void setWrenches(java.util.List<geometry_msgs.Wrench> value);
  geometry_msgs.Wrench getTotalWrench();
  void setTotalWrench(geometry_msgs.Wrench value);
  java.util.List<geometry_msgs.Vector3> getContactPositions();
  void setContactPositions(java.util.List<geometry_msgs.Vector3> value);
  java.util.List<geometry_msgs.Vector3> getContactNormals();
  void setContactNormals(java.util.List<geometry_msgs.Vector3> value);
  double[] getDepths();
  void setDepths(double[] value);
}
