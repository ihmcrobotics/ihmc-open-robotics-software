package us.ihmc.rdx.ui.graphics.ros2;

import imgui.internal.ImGui;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2CenterposeVisualizer extends RDXVisualizer implements ROS2TopicHolder<DetectedObjectPacket>
{
   private final ROS2Topic<DetectedObjectPacket> topic;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ROS2Input<DetectedObjectPacket> subscription;

   public RDXROS2CenterposeVisualizer(String title, ROS2PublishSubscribeAPI ros2, ROS2Topic<DetectedObjectPacket> topic)
   {
      super(title);
      this.topic = topic;

      subscription = ros2.subscribe(topic);
   }

   @Override
   public void update()
   {
      super.update();

      if (subscription.getMessageNotification().poll())
      {
         frequencyPlot.recordEvent();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      frequencyPlot.renderImGuiWidgets();
   }

   public ImGuiFrequencyPlot getFrequencyPlot()
   {
      return frequencyPlot;
   }

   @Override
   public ROS2Topic<DetectedObjectPacket> getTopic()
   {
      return topic;
   }
}