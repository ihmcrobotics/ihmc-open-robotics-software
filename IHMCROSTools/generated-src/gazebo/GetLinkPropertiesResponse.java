package gazebo;

public interface GetLinkPropertiesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetLinkPropertiesResponse";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose com    # center of mass location in link frame\n                          # and orientation of the moment of inertias\n                          # relative to the link frame\nbool gravity_mode         # set gravity mode on/off\nfloat64 mass              # linear mass of link\nfloat64 ixx               # moment of inertia\nfloat64 ixy               # moment of inertia\nfloat64 ixz               # moment of inertia\nfloat64 iyy               # moment of inertia\nfloat64 iyz               # moment of inertia\nfloat64 izz               # moment of inertia\nbool success              # return true if get info is successful\nstring status_message     # comments if available";
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
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}
