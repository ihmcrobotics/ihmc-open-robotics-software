package gazebo;

public interface SetLinkProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/SetLinkProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring link_name          # name of link\ngeometry_msgs/Pose com    # center of mass location in link frame\n                          # and orientation of the moment of inertias\n                          # relative to the link frame\nbool gravity_mode         # set gravity mode on/off\nfloat64 mass              # linear mass of link\nfloat64 ixx               # moment of inertia\nfloat64 ixy               # moment of inertia\nfloat64 ixz               # moment of inertia\nfloat64 iyy               # moment of inertia\nfloat64 iyz               # moment of inertia\nfloat64 izz               # moment of inertia\n---\nbool success              # return true if get info is successful\nstring status_message     # comments if available\n";
}
