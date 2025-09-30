package tf;

import geometry_msgs.TransformStamped;
import java.util.List;
import org.ros.internal.message.Message;

public interface tfMessage extends Message {
   String _TYPE = "tf/tfMessage";
   String _DEFINITION = "geometry_msgs/TransformStamped[] transforms\n";

   List<TransformStamped> getTransforms();

   void setTransforms(List<TransformStamped> var1);
}
