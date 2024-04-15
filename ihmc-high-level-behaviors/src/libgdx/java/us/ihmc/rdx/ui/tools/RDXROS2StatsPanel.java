package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImBoolean;
import us.ihmc.pubsub.stats.PubSubRateCalculator;
import us.ihmc.pubsub.stats.PubSubStats;
import us.ihmc.pubsub.stats.PubSubStatsTools;
import us.ihmc.pubsub.stats.PublisherStats;
import us.ihmc.pubsub.stats.SubscriberStats;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

import java.util.ArrayList;
import java.util.Comparator;

public class RDXROS2StatsPanel extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final PubSubRateCalculator publishFrequency = new PubSubRateCalculator();
   private final PubSubRateCalculator receiveFrequency = new PubSubRateCalculator();

   private final ImBoolean sortByLargestMessageSize = new ImBoolean(true);
   private final ImBoolean hideInactiveTopics = new ImBoolean(true);
   private final ArrayList<PublisherStats> sortedPublishers = new ArrayList<>();
   private final ArrayList<SubscriberStats> sortedSubscribers = new ArrayList<>();

   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats", null, false, true);

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.beginMenuBar())
      {
         if (ImGui.beginMenu(labels.get("Options")))
         {
            if (ImGui.menuItem(labels.get("Sort by Topic Name"), null, !sortByLargestMessageSize.get()))
            {
               sortByLargestMessageSize.set(false);
            }
            ImGui.menuItem(labels.get("Sort by Message Size"), null, sortByLargestMessageSize);
            ImGui.menuItem(labels.get("Hide Inactive Topics"), null, hideInactiveTopics);

            ImGui.endMenu();
         }

         ImGui.endMenuBar();
      }

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

      ImGuiTools.separatorText("Overall");

      if (ImGui.beginTable(labels.get("Overall"), 9, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Nodes"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publishers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Subscribers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Matched Subscriptions"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publications"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Received Messages"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publish Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Receive Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest message size"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.NUMBER_OF_PARTICIPANTS_CREATED));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.PUBLISHER_STATS.size()));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.SUBSCRIBER_STATS.size()));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.NUMBER_OF_MATCHED_SUBSCRIPTIONS));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.NUMBER_OF_PUBLISHED_MESSAGES));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(publishFrequency.finiteDifference(PubSubStats.NUMBER_OF_PUBLISHED_MESSAGES)));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(receiveFrequency.finiteDifference(PubSubStats.NUMBER_OF_RECEIVED_MESSAGES)));
         ImGui.tableNextColumn();
         ImGui.text(PubSubStatsTools.getHumanReadableDataSize(PubSubStats.LARGEST_MESSAGE_SIZE));

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Publishers");

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

         if (sortByLargestMessageSize.get())
         {
            // For when publishers get added
            if (sortedPublishers.size() < PubSubStats.PUBLISHER_STATS.size())
            {
               sortedPublishers.clear();
               sortedPublishers.addAll(PubSubStats.PUBLISHER_STATS.values());
            }

            sortedPublishers.sort(Comparator.comparing(PublisherStats::getLargestMessageSize).reversed());

            for (PublisherStats publisherStats : sortedPublishers)
               renderPublisherRow(publisherStats);
         }
         else
         {
            for (PublisherStats publisherStats : PubSubStats.PUBLISHER_STATS.values())
               renderPublisherRow(publisherStats);
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

         if (sortByLargestMessageSize.get())
         {
            // For when subscribers get added
            if (sortedSubscribers.size() < PubSubStats.SUBSCRIBER_STATS.size())
            {
               sortedSubscribers.clear();
               sortedSubscribers.addAll(PubSubStats.SUBSCRIBER_STATS.values());
            }

            sortedSubscribers.sort(Comparator.comparing(SubscriberStats::getLargestMessageSize).reversed());

            for (SubscriberStats subscriberStats : sortedSubscribers)
               renderSubscriberRow(subscriberStats);
         }
         else
         {
            for (SubscriberStats subscriberStats : PubSubStats.SUBSCRIBER_STATS.values())
               renderSubscriberRow(subscriberStats);
         }

         ImGui.endTable();
      }
   }

   private void renderSubscriberRow(SubscriberStats subscriberStats)
   {
      if (subscriberStats.getNumberOfReceivedMessages() > 0 || !hideInactiveTopics.get())
      {
         subscriberStats.update();

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
   }

   private void renderPublisherRow(PublisherStats publisherStats)
   {
      if (publisherStats.getNumberOfPublishedMessages() > 0 || !hideInactiveTopics.get())
      {
         publisherStats.update();

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
   }
}
