package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import us.ihmc.pubsub.stats.PubSubStats;
import us.ihmc.pubsub.stats.PubSubStatsTools;
import us.ihmc.pubsub.stats.PublisherStats;
import us.ihmc.pubsub.stats.SubscriberStats;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.tools.time.RateCalculator;

public class RDXROS2StatsPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RateCalculator publishFrequency = new RateCalculator(100);
   private final RateCalculator receiveFrequency = new RateCalculator(100);

   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats");

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Number of nodes created: %d".formatted(PubSubStats.NUMBER_OF_PARTICIPANTS_CREATED));
      ImGui.text("Number of publishers created: %d".formatted(PubSubStats.PUBLISHER_STATS.size()));
      ImGui.text("Number of subscribers created: %d".formatted(PubSubStats.SUBSCRIBER_STATS.size()));
      ImGui.text("Number of matched subscriptions: %d".formatted(PubSubStats.NUMBER_OF_MATCHED_SUBSCRIPTIONS));
      ImGui.text("Number of published messages: %d".formatted(PubSubStats.NUMBER_OF_PUBLISHED_MESSAGES));
      ImGui.text("Publish frequency: %.0f Hz".formatted(publishFrequency.finiteDifference(PubSubStats.NUMBER_OF_PUBLISHED_MESSAGES)));
      ImGui.text("Number of received messages: %d".formatted(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES));
      ImGui.text("Receive frequency: %.0f Hz".formatted(receiveFrequency.finiteDifference(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES)));
      ImGui.text("Largest message size: %.1f kB".formatted(PubSubStats.LARGEST_MESSAGE_SIZE / 1000.0));

      ImGuiTools.separatorText("Publishers");

      int tableFlags = ImGuiTableFlags.None;
      tableFlags += ImGuiTableFlags.Resizable;
      tableFlags += ImGuiTableFlags.SizingFixedFit;
      tableFlags += ImGuiTableFlags.Reorderable;
      tableFlags += ImGuiTableFlags.Hideable;
      // ImGui.tableGetSortSpecs is missing
      // tableFlags += ImGuiTableFlags.Sortable;
      // tableFlags += ImGuiTableFlags.SortMulti;
      tableFlags += ImGuiTableFlags.RowBg;
      tableFlags += ImGuiTableFlags.BordersOuter;
      tableFlags += ImGuiTableFlags.BordersV;
      tableFlags += ImGuiTableFlags.NoBordersInBody;
      if (ImGui.beginTable(labels.get("Publishers"), 8, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Reliability"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publications"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         for (PublisherStats publisherStats : PubSubStats.PUBLISHER_STATS.values())
         {
            ImGui.tableNextRow();

            ImGui.tableNextColumn();
            ImGui.text(publisherStats.getPublisher().getAttributes().getHumanReadableTopicName());
            ImGui.tableNextColumn();
            ImGui.text(publisherStats.getPublisher().getAttributes().getHumanReadableTopicDataTypeName());
            ImGui.tableNextColumn();
            ImGui.text(publisherStats.getPublisher().getAttributes().getReliabilityKind().name());
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(publisherStats.getNumberOfPublishedMessages()));
            ImGui.tableNextColumn();
            ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableDataSize(Math.round(publisherStats.getBandwidth()))));
            ImGui.tableNextColumn();
            ImGui.text("%.0f Hz".formatted(publisherStats.getPublishFrequency()));
            ImGui.tableNextColumn();
            ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableDataSize(publisherStats.getLargestMessageSize())));
            ImGui.tableNextColumn();
            ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableDataSize(publisherStats.getCurrentMessageSize())));
         }

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Subscribers");

      if (ImGui.beginTable(labels.get("Subscribers"), 8, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Reliability"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Received"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         for (SubscriberStats subscriberStats : PubSubStats.SUBSCRIBER_STATS.values())
         {
            ImGui.tableNextRow();

            ImGui.tableNextColumn();
            ImGui.text(subscriberStats.getSubscriber().getAttributes().getHumanReadableTopicName());
            ImGui.tableNextColumn();
            ImGui.text(subscriberStats.getSubscriber().getAttributes().getHumanReadableTopicDataTypeName());
            ImGui.tableNextColumn();
            ImGui.text(subscriberStats.getSubscriber().getAttributes().getReliabilityKind().name());
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(subscriberStats.getNumberOfReceivedMessages()));
            ImGui.tableNextColumn();
            ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableDataSize(Math.round(subscriberStats.getBandwidth()))));
            ImGui.tableNextColumn();
            ImGui.text("%.0f Hz".formatted(subscriberStats.getReceiveFrequency()));
            ImGui.tableNextColumn();
            ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableDataSize(subscriberStats.getLargestMessageSize())));
            ImGui.tableNextColumn();
            ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableDataSize(subscriberStats.getCurrentMessageSize())));
         }

         ImGui.endTable();
      }
   }
}
