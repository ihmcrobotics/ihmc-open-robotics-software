package us.ihmc.rdx.ui.graphics.ros2;

import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class RDXROS2MultiTopicVisualizer extends RDXVisualizer
{
   private final Map<ROS2Topic<?>, ImGuiAveragedFrequencyText> topicToFrequencies = new HashMap<>();

   public RDXROS2MultiTopicVisualizer(String title)
   {
      super(title);
   }

   public abstract List<ROS2Topic<?>> getTopics();

   public ImGuiAveragedFrequencyText getFrequency(ROS2Topic<?> topic)
   {
      if (!topicToFrequencies.containsKey(topic))
         topicToFrequencies.put(topic, new ImGuiAveragedFrequencyText());

      return topicToFrequencies.get(topic);
   }
}
