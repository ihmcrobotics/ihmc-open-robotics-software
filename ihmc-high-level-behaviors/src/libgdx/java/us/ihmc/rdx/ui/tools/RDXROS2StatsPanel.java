package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImBoolean;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.stats.CommonStats;
import us.ihmc.pubsub.stats.ParticipantStats;
import us.ihmc.pubsub.stats.PubSubRateCalculator;
import us.ihmc.pubsub.stats.PubSubStats;
import us.ihmc.pubsub.stats.PubSubStatsTools;
import us.ihmc.pubsub.stats.PublisherStats;
import us.ihmc.pubsub.stats.SubscriberStats;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

import java.util.Comparator;
import java.util.TreeSet;

public class RDXROS2StatsPanel extends RDXPanel
{
   private static final Comparator<CommonStats> LARGEST_MESSAGE_COMPARATOR = Comparator.comparing(CommonStats::getLargestMessageSize)
                                                                                       .reversed()
                                                                                       .thenComparingInt(Object::hashCode);

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final PubSubRateCalculator publishFrequency = new PubSubRateCalculator();
   private final PubSubRateCalculator receiveFrequency = new PubSubRateCalculator();

   private final ImBoolean sortByLargestMessageSize = new ImBoolean(true);
   private final ImBoolean hideInactiveTopics = new ImBoolean(true);
   private final TreeSet<PublisherStats> sortedPublishers = new TreeSet<>(LARGEST_MESSAGE_COMPARATOR);
   private final TreeSet<SubscriberStats> sortedSubscribers = new TreeSet<>(LARGEST_MESSAGE_COMPARATOR);

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
         ImGui.text("%d".formatted(PubSubStats.PARTICIPANT_STATS.size()));
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
         ImGui.text(PubSubStatsTools.getHumanReadableByteSize(PubSubStats.LARGEST_MESSAGE_SIZE));

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Nodes");

      if (ImGui.beginTable(labels.get("Nodes"), 4, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publishers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Subscribers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Removed"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         ImGui.tableNextRow();

         for (ParticipantStats participantStats : PubSubStats.PARTICIPANT_STATS.values())
         {
            ImGui.tableNextColumn();
            ImGui.text(participantStats.getParticipant().getAttributes().getName());
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(participantStats.getPublishers().size()));
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(participantStats.getSubscribers().size()));
            ImGui.tableNextColumn();
            ImGui.text("%b".formatted(participantStats.getRemoved()));
         }

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Publishers");

      if (ImGui.beginTable(labels.get("Publishers"), 10, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Reliability"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publications"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupColumn(labels.get("Removed"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         if (sortByLargestMessageSize.get())
         {
            sortedPublishers.clear();
            sortedPublishers.addAll(PubSubStats.PUBLISHER_STATS.values());

            for (PublisherStats publisherStats : sortedPublishers)
               renderPublisherRow(publisherStats);
         }
         else
         {
            for (ParticipantStats participant : PubSubStats.PARTICIPANT_STATS.values())
               for (Publisher publisher : participant.getPublishers())
                  renderPublisherRow(PubSubStats.PUBLISHER_STATS.get(publisher));
         }

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Subscribers");

      if (ImGui.beginTable(labels.get("Subscribers"), 11, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Reliability"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Received"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupColumn(labels.get("Matched"));
         ImGui.tableSetupColumn(labels.get("Removed"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         if (sortByLargestMessageSize.get())
         {
            sortedSubscribers.clear();
            sortedSubscribers.addAll(PubSubStats.SUBSCRIBER_STATS.values());

            for (SubscriberStats subscriberStats : sortedSubscribers)
               renderSubscriberRow(subscriberStats);
         }
         else
         {
            for (ParticipantStats participantStats : PubSubStats.PARTICIPANT_STATS.values())
               for (Subscriber<?> subscriber : participantStats.getSubscribers())
                  renderSubscriberRow(PubSubStats.SUBSCRIBER_STATS.get(subscriber));
         }

         ImGui.endTable();
      }
   }

   private void renderPublisherRow(PublisherStats publisherStats)
   {
      if (publisherStats.getNumberOfPublishedMessages() > 0 || !hideInactiveTopics.get())
      {
         publisherStats.update();

         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text(publisherStats.getParticipant().getAttributes().getName());
         ImGui.tableNextColumn();
         ImGui.text(publisherStats.getPublisher().getAttributes().getHumanReadableTopicName());
         ImGui.tableNextColumn();
         ImGui.text(publisherStats.getPublisher().getAttributes().getHumanReadableTopicDataTypeName());
         ImGui.tableNextColumn();
         ImGui.text(publisherStats.getPublisher().getAttributes().getReliabilityKind().name());
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(publisherStats.getNumberOfPublishedMessages()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(publisherStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(publisherStats.getPublishFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(publisherStats.getRemoved()));
      }
   }

   private void renderSubscriberRow(SubscriberStats subscriberStats)
   {
      if (subscriberStats.getNumberOfReceivedMessages() > 0 || !hideInactiveTopics.get())
      {
         subscriberStats.update();

         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text(subscriberStats.getParticipant().getAttributes().getName());
         ImGui.tableNextColumn();
         ImGui.text(subscriberStats.getSubscriber().getAttributes().getHumanReadableTopicName());
         ImGui.tableNextColumn();
         ImGui.text(subscriberStats.getSubscriber().getAttributes().getHumanReadableTopicDataTypeName());
         ImGui.tableNextColumn();
         ImGui.text(subscriberStats.getSubscriber().getAttributes().getReliabilityKind().name());
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(subscriberStats.getNumberOfReceivedMessages()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(subscriberStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(subscriberStats.getReceiveFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(subscriberStats.getHasMatched()));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(subscriberStats.getRemoved()));
      }
   }
}
