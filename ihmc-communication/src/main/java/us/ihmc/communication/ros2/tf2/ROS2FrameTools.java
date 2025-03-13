package us.ihmc.communication.ros2.tf2;

import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.ros2.ROS2Topic;

public class ROS2FrameTools
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().withModule("tf").withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().withModule("tf_static").withType(TFMessage.class);

   public static final String WORLD_FRAME_ID = "world";

   private static final ROS2StaticFrame WORLD_FRAME = ROS2StaticFrame.constructARootFrame(WORLD_FRAME_ID);

   public static synchronized ROS2StaticFrame getWorldFrame()
   {
      return WORLD_FRAME;
   }
}
