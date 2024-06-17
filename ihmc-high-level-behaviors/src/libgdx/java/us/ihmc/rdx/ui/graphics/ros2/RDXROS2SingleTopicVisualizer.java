package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;

public abstract class RDXROS2SingleTopicVisualizer<T> extends RDXVisualizer
{
   private final ImGuiAveragedFrequencyText frequencyText = new ImGuiAveragedFrequencyText();

   public RDXROS2SingleTopicVisualizer(String title)
   {
      super(title);
   }

   public abstract ROS2Topic<T> getTopic();

   public ImGuiAveragedFrequencyText getFrequency()
   {
      return frequencyText;
   }
}
