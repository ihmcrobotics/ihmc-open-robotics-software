package lidar_obstacle_detection;

import org.ros.internal.message.Message;
import java.util.List;


public interface GDXBoxesMessage extends Message {
   static final java.lang.String _TYPE = "lidar_obstacle_detection/GDXBoxesMessage";
   static final java.lang.String _DEFINITION =
         "lidar_obstacle_detection/GDXBoxMessage[] boxes\n";
   List<GDXBoxMessage>  getBoxes();
   void setBoxes(List<GDXBoxMessage> value);
}
