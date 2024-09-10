package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.ffmpeg.global.avutil;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.perception.streaming.StreamingTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;

public class RDXROS2SRTVideoStreamVisualizer extends RDXROS2OpenCVVideoVisualizer<SRTStreamMessage>
{
   private final ROS2IOTopicPair<SRTStreamMessage> streamTopic;
   private final ROS2SRTVideoSubscriber subscriber;

   public RDXROS2SRTVideoStreamVisualizer(String title, ROS2IOTopicPair<SRTStreamMessage> streamTopic)
   {
      super(title, title, false);

      this.streamTopic = streamTopic;
      String nodeName = title.toLowerCase().replaceAll("\\W+", "_") + "_srt_stream_visualizer";
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, nodeName);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      subscriber = new ROS2SRTVideoSubscriber(ros2Helper, streamTopic, StreamingTools.getMyAddress(), avutil.AV_PIX_FMT_RGBA);

      setActivenessChangeCallback(isActive ->
      {
         if (isActive)
            // TODO: Kill this thread on unsubscribe (ExecutorService instead?)
           ThreadTools.startAsDaemon(subscriber::subscribe, title + "Subscriber");
         else
            subscriber.unsubscribe();
      });
   }

   @Override
   public void update()
   {
      super.update();
      if (subscriber.isConnected())
      {
         subscriber.update();
         getFrequency().ping();
         if (subscriber.hasCameraIntrinsics() && subscriber.hasReceivedFirstFrame())
            updateImage();
      }
      getOpenCVVideoVisualizer().setActive(isActive());
      getOpenCVVideoVisualizer().update();
   }

   private void updateImage()
   {
      getOpenCVVideoVisualizer().doReceiveMessageOnThread(() ->
      {
         CameraIntrinsics imageIntrinsics = subscriber.getCameraIntrinsics();
         getOpenCVVideoVisualizer().updateImageDimensions(imageIntrinsics.getWidth(), imageIntrinsics.getHeight());
         subscriber.getCurrentFrame().copyTo(getOpenCVVideoVisualizer().getRGBA8Mat());
      });
   }

   @Nullable
   @Override
   public RDXPanel getPanel()
   {
      return getOpenCVVideoVisualizer().getPanel();
   }

   @Override
   public void renderImGuiWidgets()
   {
      getOpenCVVideoVisualizer().renderImGuiWidgets();
   }

   @Override
   public ROS2Topic<SRTStreamMessage> getTopic()
   {
      return streamTopic.getStatusTopic();
   }

   @Override
   public void destroy()
   {
      super.destroy();
      getOpenCVVideoVisualizer().destroy();
      subscriber.unsubscribe();
      subscriber.destroy();
   }
}
