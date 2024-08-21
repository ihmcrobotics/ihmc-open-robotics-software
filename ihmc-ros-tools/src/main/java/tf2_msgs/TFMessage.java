package tf2_msgs;

import geometry_msgs.TransformStamped;
import java.util.List;
import org.ros.internal.message.Message;

public interface TFMessage extends Message {
   String _TYPE = "tf2_msgs/TFMessage";
   String _DEFINITION = "geometry_msgs/TransformStamped[] transforms\n";

   List<TransformStamped> getTransforms();

   void setTransforms(List<TransformStamped> var1);
}