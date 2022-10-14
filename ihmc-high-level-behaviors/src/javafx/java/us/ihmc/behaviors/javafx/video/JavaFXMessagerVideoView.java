package us.ihmc.behaviors.javafx.video;

import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;

public class JavaFXMessagerVideoView extends JavaFXVideoView
{
   // necessary to remove listener
   private final TopicListener<VideoPacket> acceptVideoTopicListener = videoPacket -> acceptVideo(videoPacket.getData());

   private Messager messager;
   private Topic<VideoPacket> topic;

   public JavaFXMessagerVideoView(Messager messager, Topic<VideoPacket> topic, int width, int height, boolean flipX, boolean flipY)
   {
      super(width, height, flipX, flipY);

      this.messager = messager;
      this.topic = topic;
   }

   public void start()
   {
      messager.registerTopicListener(topic, acceptVideoTopicListener);
      super.start();
   }

   public void stop()
   {
      messager.removeTopicListener(topic, acceptVideoTopicListener);
      super.stop();
   }

   public void destroy()
   {
      messager.removeTopicListener(topic, acceptVideoTopicListener);
      super.destroy();
   }
}
