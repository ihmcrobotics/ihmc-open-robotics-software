package nav_msgs;

import geometry_msgs.PoseWithCovariance;
import geometry_msgs.TwistWithCovariance;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Odometry extends Message {
   String _TYPE = "nav_msgs/Odometry";
   String _DEFINITION = "# This represents an estimate of a position and velocity in free space.  \n# The pose in this message should be specified in the coordinate frame given by header.frame_id.\n# The twist in this message should be specified in the coordinate frame given by the child_frame_id\nHeader header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n";

   Header getHeader();

   void setHeader(Header var1);

   String getChildFrameId();

   void setChildFrameId(String var1);

   PoseWithCovariance getPose();

   void setPose(PoseWithCovariance var1);

   TwistWithCovariance getTwist();

   void setTwist(TwistWithCovariance var1);
}