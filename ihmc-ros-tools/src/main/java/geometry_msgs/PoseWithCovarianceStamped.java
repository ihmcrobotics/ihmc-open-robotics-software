package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface PoseWithCovarianceStamped extends Message {
   String _TYPE = "geometry_msgs/PoseWithCovarianceStamped";
   String _DEFINITION = "# This expresses an estimated pose with a reference coordinate frame and timestamp\n\nHeader header\nPoseWithCovariance pose\n";

   Header getHeader();

   void setHeader(Header var1);

   PoseWithCovariance getPose();

   void setPose(PoseWithCovariance var1);
}