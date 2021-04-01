package lidar_obstacle_detection;

public interface GDXBoxesMessage extends org.ros.internal.message.Message {
   static final java.lang.String _TYPE = "lidar_obstacle_detection/GDXBoxesMessage";
   static final java.lang.String _DEFINITION =
         "lidar_obstacle_detection/GDXBoxMessage[] boxes\n";
   java.util.List<GDXBoxMessage>  getBoundingBoxes();
   void setBoundingBoxes(java.util.List<GDXBoxMessage> value);
}
