package us.ihmc.humanoidBehaviors.ui.video;

import controller_msgs.msg.dds.Image32;
import controller_msgs.msg.dds.VideoPacket;
import rcl_interfaces.msg.dds.Log;
import sensor_msgs.msg.dds.CompressedImage;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

public class JavaFXROS2VideoView extends JavaFXVideoView
{
   private final ROS2NodeInterface ros2Node;
   private final ROS2Topic<?> topic;
   private IHMCROS2Callback<?> ros2Callback;

   int i = 0;

   public JavaFXROS2VideoView(ROS2NodeInterface ros2Node, ROS2Topic<?> topic, int width, int height, boolean flipX, boolean flipY)
   {
      super(width, height, flipX, flipY);

      this.ros2Node = ros2Node;
      this.topic = topic;
   }

   @Override
   public void start()
   {
      LogTools.info("Subscribing to {}", topic.getName());
      ros2Callback = new IHMCROS2Callback<>(ros2Node, topic, message ->
      {
         if (message instanceof VideoPacket)
         {
            acceptVideo(((VideoPacket) message).getData());
         }
         else if (message instanceof Image)
         {
//            if (i++ % 30 == 0)
//            {
//               LogTools.info("Received # {}", i);
               acceptVideo(((Image) message).getData());
//            }
         }
         else if (message instanceof CompressedImage)
         {
//            if (i++ % 30 == 0)
//            {
//               LogTools.info("Received compressed # {}", i);
               acceptVideo(((CompressedImage) message).getData());
//            }
         }
//         else if (message instanceof Image32)
//         {
//            IDLSequence.Byte();
//            acceptVideo(((Image32) message).getRgbdata());
//         }
      });
      super.start();
   }

   @Override
   public void stop()
   {
      ros2Callback.destroy();
      super.stop();
   }

   @Override
   public void destroy()
   {
      ros2Callback.destroy();
      super.destroy();
   }
}
