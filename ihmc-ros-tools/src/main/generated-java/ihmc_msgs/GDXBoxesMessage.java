package ihmc_msgs;

public interface GDXBoxesMessage extends org.ros.internal.message.Message {
   static final java.lang.String _TYPE = "ihmc_msgs/GDXBoxesMessage";
   static final java.lang.String _DEFINITION = "## GDXBoxesMessage\n# This message is used to visualize the bounding box of the pointcloud\n\n\n";
   java.util.List<ihmc_msgs.GDXBoxMessage>  getBoundingBoxes();
   void setBoundingBoxes(java.util.List<ihmc_msgs.GDXBoxMessage> value);
}
