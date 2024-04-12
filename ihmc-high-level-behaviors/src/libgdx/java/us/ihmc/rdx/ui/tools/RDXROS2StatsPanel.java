package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import us.ihmc.pubsub.impl.PubSubStats;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.ros2.ROS2Stats;
import us.ihmc.tools.time.RateCalculator;

public class RDXROS2StatsPanel extends RDXPanel
{
   private final RateCalculator publishFrequency = new RateCalculator(100);
   private final RateCalculator receiveFrequency = new RateCalculator(100);

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
      ImGui.text("Number of matched subscriptions: %d".formatted(PubSubStats.NUMBER_OF_MATCHED_SUBSCRIPTIONS));
      ImGui.text("Number of published messages: %d".formatted(ROS2Stats.NUMBER_OF_PUBLISHED_MESSAGES));
      ImGui.text("Publish frequency: %.0f Hz".formatted(publishFrequency.finiteDifference(ROS2Stats.NUMBER_OF_PUBLISHED_MESSAGES)));
      ImGui.text("Number of received messages: %d".formatted(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES));
      ImGui.text("Receive frequency: %.0f Hz".formatted(receiveFrequency.finiteDifference(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES)));
   }
}
