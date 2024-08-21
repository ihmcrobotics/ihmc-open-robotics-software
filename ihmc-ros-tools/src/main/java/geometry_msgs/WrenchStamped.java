package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface WrenchStamped extends Message {
   String _TYPE = "geometry_msgs/WrenchStamped";
   String _DEFINITION = "# A wrench with reference coordinate frame and timestamp\nHeader header\nWrench wrench\n";

   Header getHeader();

   void setHeader(Header var1);

   Wrench getWrench();

   void setWrench(Wrench var1);
}