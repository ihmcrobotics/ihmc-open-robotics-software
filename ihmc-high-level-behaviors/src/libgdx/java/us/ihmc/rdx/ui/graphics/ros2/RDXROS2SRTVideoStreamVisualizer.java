package us.ihmc.rdx.ui.graphics.ros2;

import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;

public class RDXROS2SRTVideoStreamVisualizer extends RDXROS2OpenCVVideoVisualizer<SRTStreamStatus>
{
   private final ROS2Topic<SRTStreamStatus> streamTopic;
   private final ROS2SRTVideoSubscriber subscriber;

   public RDXROS2SRTVideoStreamVisualizer(ROS2PublishSubscribeAPI ros2, String title, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      super(title, title, false);

      this.streamTopic = streamTopic;

      subscriber = new ROS2SRTVideoSubscriber(ros2, streamTopic, avutil.AV_PIX_FMT_RGBA);
      subscriber.addNewFrameConsumer(this::updateImage);

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
      getOpenCVVideoVisualizer().setActive(isActive());
      getOpenCVVideoVisualizer().update();
   }

   private void updateImage(Mat newImage)
   {
      getFrequency().ping();
      CameraIntrinsics imageIntrinsics = subscriber.getCameraIntrinsics();
      getOpenCVVideoVisualizer().updateImageDimensions(imageIntrinsics.getWidth(), imageIntrinsics.getHeight());
      newImage.copyTo(getOpenCVVideoVisualizer().getRGBA8Mat());
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
   public ROS2Topic<SRTStreamStatus> getTopic()
   {
      return streamTopic;
   }

   @Override
   public void destroy()
   {
      subscriber.destroy();
      super.destroy();
      getOpenCVVideoVisualizer().destroy();
   }
}
