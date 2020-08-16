package us.ihmc.humanoidBehaviors.ui.video;

import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2NodeInterface;

public class JavaFXROS2VideoView extends JavaFXVideoView
{
   private final Ros2NodeInterface ros2Node;
   private final ROS2Topic<VideoPacket> topic;
   private IHMCROS2Callback<VideoPacket> ros2Callback;

   public JavaFXROS2VideoView(Ros2NodeInterface ros2Node, ROS2Topic<VideoPacket> topic, int width, int height, boolean flipX, boolean flipY)
   {
      super(width, height, flipX, flipY);

      this.ros2Node = ros2Node;
      this.topic = topic;
   }

   @Override
   public void start()
   {
      ros2Callback = new IHMCROS2Callback<>(ros2Node, topic, this::acceptVideo);
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
