package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.ffmpeg.global.avutil;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.perception.streaming.StreamingTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;

public class RDXROS2SRTVideoStreamVisualizer extends RDXROS2OpenCVVideoVisualizer<SRTStreamMessage>
{
   private final ROS2IOTopicPair<SRTStreamMessage> streamTopic;
   private final ROS2SRTVideoSubscriber subscriber;

   public RDXROS2SRTVideoStreamVisualizer(ROS2PublishSubscribeAPI ros2, String title, ROS2IOTopicPair<SRTStreamMessage> streamTopic)
   {
      super(title, title, false);

      this.streamTopic = streamTopic;

      subscriber = new ROS2SRTVideoSubscriber(ros2, streamTopic, StreamingTools.getMyAddress(), avutil.AV_PIX_FMT_RGBA);

      setActivenessChangeCallback(isActive ->
      {
         if (isActive)
            subscriber.subscribe();
         else
            subscriber.unsubscribe();
      });
   }

   @Override
   public void update()
   {
      super.update();
      updateImage();
      getOpenCVVideoVisualizer().setActive(isActive());
      getOpenCVVideoVisualizer().update();
   }

   private void updateImage()
   {
      getOpenCVVideoVisualizer().doReceiveMessageOnThread(() ->
      {
         if (subscriber.isConnected())
         {
            subscriber.update();
            getFrequency().ping();
            if (subscriber.hasCameraIntrinsics() && subscriber.newFrameAvailable())
            {
               CameraIntrinsics imageIntrinsics = subscriber.getCameraIntrinsics();
               getOpenCVVideoVisualizer().updateImageDimensions(imageIntrinsics.getWidth(), imageIntrinsics.getHeight());
               subscriber.getCurrentFrame().copyTo(getOpenCVVideoVisualizer().getRGBA8Mat());
            }
         }
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
      subscriber.destroy();
   }
}
