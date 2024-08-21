//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package geometry_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface TransformStamped extends Message {
   String _TYPE = "geometry_msgs/TransformStamped";
   String _DEFINITION = "# This expresses a transform from coordinate frame header.frame_id\n# to the coordinate frame child_frame_id\n#\n# This message is mostly used by the \n# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. \n# See its documentation for more information.\n\nHeader header\nstring child_frame_id # the frame id of the child frame\nTransform transform\n";

   Header getHeader();

   void setHeader(Header var1);

   String getChildFrameId();

   void setChildFrameId(String var1);

   Transform getTransform();

   void setTransform(Transform var1);
}
