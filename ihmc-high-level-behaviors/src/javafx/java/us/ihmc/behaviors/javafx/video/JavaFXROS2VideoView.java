package us.ihmc.behaviors.javafx.video;

import perception_msgs.msg.dds.VideoPacket;
import sensor_msgs.msg.dds.CompressedImage;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

public class JavaFXROS2VideoView extends JavaFXVideoView
{
   private final ROS2NodeInterface ros2Node;
   private final ROS2Topic<?> topic;
   private IHMCROS2Callback<?> ros2Callback;

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
      ros2Callback = new IHMCROS2Callback<>(ros2Node, topic, ROS2QosProfile.BEST_EFFORT(), message ->
      {
         if (message instanceof VideoPacket)
         {
            acceptVideo(((VideoPacket) message).getData());
         }
         else if (message instanceof Image)
         {
            acceptVideo(((Image) message).getData());
         }
         else if (message instanceof CompressedImage)
         {
            acceptVideo(((CompressedImage) message).getData());
         }
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
