package gazebo;

public interface ContactState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/ContactState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring info                                   # text info on this contact\nstring geom1_name                             # name of contact geom1\nstring geom2_name                             # name of contact geom2\ngeometry_msgs/Wrench[] wrenches               # list of forces/torques\ngeometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF\ngeometry_msgs/Vector3[] contact_positions     # list of contact position\ngeometry_msgs/Vector3[] contact_normals       # list of contact normals\nfloat64[] depths                              # list of penetration depths\n";
  java.lang.String getInfo();
  void setInfo(java.lang.String value);
  java.lang.String getGeom1Name();
  void setGeom1Name(java.lang.String value);
  java.lang.String getGeom2Name();
  void setGeom2Name(java.lang.String value);
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
