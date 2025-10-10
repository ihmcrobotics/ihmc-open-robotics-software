package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PoseStamped extends Message {
   String _TYPE = "geometry_msgs/PoseStamped";
   String _DEFINITION = "# A Pose with reference coordinate frame and timestamp\nHeader header\nPose pose\n";

   Header getHeader();

   void setHeader(Header var1);

   Pose getPose();

   void setPose(Pose var1);
}