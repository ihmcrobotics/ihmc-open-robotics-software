package gazebo;

public interface SetLinkPropertiesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetLinkPropertiesRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring link_name          # name of link\ngeometry_msgs/Pose com    # center of mass location in link frame\n                          # and orientation of the moment of inertias\n                          # relative to the link frame\nbool gravity_mode         # set gravity mode on/off\nfloat64 mass              # linear mass of link\nfloat64 ixx               # moment of inertia\nfloat64 ixy               # moment of inertia\nfloat64 ixz               # moment of inertia\nfloat64 iyy               # moment of inertia\nfloat64 iyz               # moment of inertia\nfloat64 izz               # moment of inertia\n";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  geometry_msgs.Pose getCom();
  void setCom(geometry_msgs.Pose value);
  boolean getGravityMode();
  void setGravityMode(boolean value);
  double getMass();
  void setMass(double value);
  double getIxx();
  void setIxx(double value);
  double getIxy();
  void setIxy(double value);
  double getIxz();
  void setIxz(double value);
  double getIyy();
  void setIyy(double value);
  double getIyz();
  void setIyz(double value);
  double getIzz();
  void setIzz(double value);
}
