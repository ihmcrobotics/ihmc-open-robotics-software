package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Stats;
import us.ihmc.tools.time.RateCalculator;

public class RDXROS2StatsPanel extends RDXPanel
{
   private final RateCalculator publishFrequency = new RateCalculator(100);

   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats");

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Number of nodes created: %d".formatted(ROS2Stats.NUMBER_OF_NODES_CREATED));
      ImGui.text("Number of publishers created: %d".formatted(ROS2Stats.NUMBER_OF_PUBLISHERS_CREATED));
      ImGui.text("Number of subscribers created: %d".formatted(ROS2Stats.NUMBER_OF_SUBSCRIBERS_CREATED));
      ImGui.text("Number of published messages: %d".formatted(ROS2Stats.NUMBER_OF_PUBLISHED_MESSAGES));

      ImGui.text("Publish frequency: %.0f Hz".formatted(publishFrequency.finiteDifference(ROS2Stats.NUMBER_OF_PUBLISHED_MESSAGES)));
   }
}
