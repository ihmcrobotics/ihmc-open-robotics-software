package us.ihmc.rdx.ui.graphics.ros2;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import perception_msgs.SRTStreamStatus;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.jros2.ROS2Topic;

public class RDXROS2SRTVideoStreamVisualizer extends RDXROS2ImageVisualizer<SRTStreamStatus>
{
   private static final String DELAY_PLOT_TEXT = "Delay (ms)";

   private final ROS2Topic<SRTStreamStatus> streamTopic;
   private final ROS2SRTVideoSubscriber subscriber;

   private float alphaFilteredDelayMS = 0.0f;
   private final ImGuiPlot delayPlot = new ImGuiPlot(DELAY_PLOT_TEXT, 1000, -1, 20);

   public RDXROS2SRTVideoStreamVisualizer(ROS2Helper ros2, String title, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      super(title, title, false);

      this.streamTopic = streamTopic;

      subscriber = new ROS2SRTVideoSubscriber(ros2, streamTopic, PixelFormat.RGBA8);
      subscriber.addNewFrameConsumer(this::updateImage);

      addActivenessChangeCallback(isActive ->
      {
         if (isActive)
            subscriber.subscribe();
         else
            subscriber.unsubscribe();
      });
   }

   private void updateImage(RawImage newImage)
   {
      getFrequency().ping();
      submitImageUpdate(imageVisualizer -> imageVisualizer.setImage(newImage));
      float delayMS = (float) Conversions.secondsToMilliseconds(subscriber.getLastFrameDelay());
      alphaFilteredDelayMS = 0.1f * delayMS + 0.9f * alphaFilteredDelayMS;
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor(alphaFilteredDelayMS, 50.0f, 200.0f));
      delayPlot.setWidth((int) (ImGui.getColumnWidth() - ImGuiTools.calcTextSizeX(DELAY_PLOT_TEXT)));
      delayPlot.render(alphaFilteredDelayMS);
      ImGui.popStyleColor();

      super.renderImGuiWidgets();
   }

   @Override
   public ROS2Topic<SRTStreamStatus> getTopic()
   {
      return streamTopic;
   }

   @Override
   public void destroy()
   {
      super.destroy();
      subscriber.destroy();
   }
}
