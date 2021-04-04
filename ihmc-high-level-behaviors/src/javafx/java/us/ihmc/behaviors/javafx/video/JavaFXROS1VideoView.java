package us.ihmc.behaviors.javafx.video;

import sensor_msgs.CompressedImage;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class JavaFXROS1VideoView extends JavaFXVideoView
{
   private final RosMainNode ros1Node;
   private final String topic;

   public JavaFXROS1VideoView(RosMainNode ros1Node, String topic, int width, int height, boolean flipX, boolean flipY)
   {
      super(width, height, flipX, flipY);

      this.ros1Node = ros1Node;
      this.topic = topic;
   }

   @Override
   public void start()
   {
      LogTools.info("Subscribing ROS 1: {}", topic);
      ros1Node.attachSubscriber(topic, CompressedImage.class, message ->
      {
         try
         {
            acceptVideo(RosTools.bufferedImageFromRosMessageJpeg(message));
         }
         catch (Exception e)
         {
            LogTools.error(e.getMessage());
            e.printStackTrace();
         }
      });
      super.start();
   }

   @Override
   public void stop()
   {
      super.stop();
   }

   @Override
   public void destroy()
   {
      super.destroy();
   }
}
