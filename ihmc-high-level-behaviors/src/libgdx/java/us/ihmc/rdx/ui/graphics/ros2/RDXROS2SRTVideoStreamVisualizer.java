package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;

public class RDXROS2SRTVideoStreamVisualizer extends RDXROS2OpenCVVideoVisualizer<SRTStreamStatus>
{
   private static final String DELAY_PLOT_TEXT = "Delay (ms)";

   private final ROS2Topic<SRTStreamStatus> streamTopic;
   private final ROS2SRTVideoSubscriber subscriber;

   private float alphaFilteredDelayMS = 0.0f;
   private final ImGuiPlot delayPlot = new ImGuiPlot(DELAY_PLOT_TEXT, 1000, -1, 20);

   public RDXROS2SRTVideoStreamVisualizer(ROS2PublishSubscribeAPI ros2, String title, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      super(title, title, false);

      this.streamTopic = streamTopic;

      subscriber = new ROS2SRTVideoSubscriber(ros2, streamTopic, PixelFormat.RGBA8);
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

   private void updateImage(RawImage newImage)
   {
      getFrequency().ping();
      getOpenCVVideoVisualizer().updateImageDimensions(newImage.getWidth(), newImage.getHeight());
      newImage.getCpuImageMat().copyTo(getOpenCVVideoVisualizer().getRGBA8Mat());
      float delayMS = (float) Conversions.secondsToMilliseconds(subscriber.getLastFrameDelay());
      alphaFilteredDelayMS = 0.1f * delayMS + 0.9f * alphaFilteredDelayMS;
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
      ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor(alphaFilteredDelayMS, 50.0f, 200.0f));
      delayPlot.setWidth((int) (ImGui.getColumnWidth() - ImGuiTools.calcTextSizeX(DELAY_PLOT_TEXT)));
      delayPlot.render(alphaFilteredDelayMS);
      ImGui.popStyleColor();

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
