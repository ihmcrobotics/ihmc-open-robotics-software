package octomap_msgs;

public interface GetOctomapResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "octomap_msgs/GetOctomapResponse";
  static final java.lang.String _DEFINITION = "octomap_msgs/Octomap map";
  octomap_msgs.Octomap getMap();
  void setMap(octomap_msgs.Octomap value);
}
