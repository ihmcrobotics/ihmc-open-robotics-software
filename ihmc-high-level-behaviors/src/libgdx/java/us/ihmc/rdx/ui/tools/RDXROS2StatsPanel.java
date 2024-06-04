package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImBoolean;
import us.ihmc.pubsub.impl.fastRTPS.FastRTPSDomain;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.TreeSet;

public class RDXROS2StatsPanel extends RDXPanel
{
   private long numberOfMatchedSubscriptions;
   private long numberOfPublications;
   private long numberOfReceivedMessages;
   private long largestMessageSize;
   private final PubSubRateCalculator publishFrequency = new PubSubRateCalculator();
   private final PubSubRateCalculator receiveFrequency = new PubSubRateCalculator();
   private double totalOutgoingBandwidth;
   private double totalIncomingBandwidth;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean sortByLargestMessageSize = new ImBoolean(true);
   private final ImBoolean hideInactiveTopics = new ImBoolean(true);

   private final HashMap<Publisher, PubSubPublisherStats> publisherStatsMap = new HashMap<>();
   private final HashMap<Subscriber<?>, PubSubSubscriberStats> subscriberStatsMap = new HashMap<>();
   private final TreeSet<Participant> participantsSortedByName = new TreeSet<>(Comparator.<Participant, String>comparing(o -> o.getAttributes().getName())
                                                                                         .thenComparingInt(Object::hashCode));
   private final TreeSet<PubSubPublisherStats> publishersSortedBySize
         = new TreeSet<>(Comparator.<PubSubPublisherStats, Long>comparing(o -> o.getPublisher().getLargestMessageSize())
                                   .reversed()
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<PubSubSubscriberStats> subscribersSortedBySize
         = new TreeSet<>(Comparator.<PubSubSubscriberStats, Long>comparing(o -> o.getSubscriber().getLargestMessageSize())
                                   .reversed()
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<PubSubPublisherStats> publishersSortedByName
         = new TreeSet<>(Comparator.<PubSubPublisherStats, String>comparing(o -> o.getPublisher().getAttributes().getHumanReadableTopicName())
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<PubSubSubscriberStats> subscribersSortedByName
         = new TreeSet<>(Comparator.<PubSubSubscriberStats, String>comparing(o -> o.getSubscriber().getAttributes().getHumanReadableTopicName())
                                   .thenComparingInt(Object::hashCode));

   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats", null, false, true);

      setRenderMethod(this::renderImGuiWidgets);
   }

   private void renderImGuiWidgets()
   {
      numberOfMatchedSubscriptions = 0;
      numberOfPublications = 0;
      numberOfReceivedMessages = 0;
      largestMessageSize = 0;
      totalOutgoingBandwidth = 0.0;
      totalIncomingBandwidth = 0.0;

      participantsSortedByName.clear();
      publishersSortedBySize.clear();
      subscribersSortedBySize.clear();
      publishersSortedByName.clear();
      subscribersSortedByName.clear();

      List<Participant> participants = FastRTPSDomain.accessInstance().getAllParticipantsForStatistics();
      synchronized (participants)
      {
         for (Participant participant : participants)
         {
            participantsSortedByName.add(participant);

            List<Publisher> publishers = participant.getAllPublishersForStatistics();
            synchronized (publishers)
            {
               for (Publisher publisher : publishers)
               {
                  PubSubPublisherStats publisherStats = publisherStatsMap.get(publisher);

                  if (publisherStats == null)
                  {
                     publisherStats = new PubSubPublisherStats(participant, publisher);
                     publisherStatsMap.put(publisher, publisherStats);
                  }

                  if (sortByLargestMessageSize.get())
                     publishersSortedBySize.add(publisherStats);
                  else
                     publishersSortedByName.add(publisherStats);
               }
            }

            List<Subscriber<?>> subscribers = participant.getAllSubscribersForStatistics();
            synchronized (subscribers)
            {
               for (Subscriber<?> subscriber : subscribers)
               {
                  PubSubSubscriberStats subscriberStats = subscriberStatsMap.get(subscriber);

                  if (subscriberStats == null)
                  {
                     subscriberStats = new PubSubSubscriberStats(participant, subscriber);
                     subscriberStatsMap.put(subscriber, subscriberStats);
                  }

                  if (sortByLargestMessageSize.get())
                     subscribersSortedBySize.add(subscriberStats);
                  else
                     subscribersSortedByName.add(subscriberStats);
               }
            }
         }
      }

      for (PubSubPublisherStats publisherStats : publisherStatsMap.values())
      {
         publisherStats.update();

         numberOfPublications += publisherStats.getPublisher().getNumberOfPublications();
         totalOutgoingBandwidth += publisherStats.getBandwidth();

         if (publisherStats.getPublisher().getLargestMessageSize() > largestMessageSize)
            largestMessageSize = publisherStats.getPublisher().getLargestMessageSize();
      }

      for (PubSubSubscriberStats subscriberStats : subscriberStatsMap.values())
      {
         subscriberStats.update();

         numberOfMatchedSubscriptions += subscriberStats.getSubscriber().hasMatched() ? 1 : 0;
         numberOfReceivedMessages += subscriberStats.getSubscriber().getNumberOfReceivedMessages();
         totalIncomingBandwidth += subscriberStats.getBandwidth();

         if (subscriberStats.getSubscriber().getLargestMessageSize() > largestMessageSize)
            largestMessageSize = subscriberStats.getSubscriber().getLargestMessageSize();
      }

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

      if (ImGui.beginTable(labels.get("Overall"), 11, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Nodes"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publishers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Subscribers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Matched Subscriptions"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publications"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Received Messages"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publish Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Receive Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Outgoing Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Incoming Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest message size"));
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(participantsSortedByName.size()));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(publisherStatsMap.size()));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(subscriberStatsMap.size()));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(numberOfMatchedSubscriptions));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(numberOfPublications));
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(numberOfReceivedMessages));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(publishFrequency.finiteDifference(numberOfPublications)));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(receiveFrequency.finiteDifference(numberOfReceivedMessages)));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(totalOutgoingBandwidth))));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(totalIncomingBandwidth))));
         ImGui.tableNextColumn();
         ImGui.text(PubSubStatsTools.getHumanReadableByteSize(largestMessageSize));

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

         for (Participant participant : participantsSortedByName)
         {
            ImGui.tableNextColumn();
            ImGui.text(participant.getAttributes().getName());
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(participant.getAllPublishersForStatistics().size()));
            ImGui.tableNextColumn();
            ImGui.text("%d".formatted(participant.getAllSubscribersForStatistics().size()));
            ImGui.tableNextColumn();
            ImGui.text("%b".formatted(participant.isRemoved()));
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
            for (PubSubPublisherStats publisherStats : publishersSortedBySize)
               renderPublisherRow(publisherStats);
         else
            for (PubSubPublisherStats publisherStats : publishersSortedByName)
               renderPublisherRow(publisherStats);

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
            for (PubSubSubscriberStats subscriberStats : subscribersSortedBySize)
               renderSubscriberRow(subscriberStats);
         else
            for (PubSubSubscriberStats subscriberStats : subscribersSortedByName)
               renderSubscriberRow(subscriberStats);

         ImGui.endTable();
      }
   }

   private void renderPublisherRow(PubSubPublisherStats publisherStats)
   {
      if (publisherStats.getPublisher().getNumberOfPublications() > 0 || !hideInactiveTopics.get())
      {
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
         ImGui.text("%d".formatted(publisherStats.getPublisher().getNumberOfPublications()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(publisherStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(publisherStats.getPublishFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getPublisher().getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getPublisher().getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(publisherStats.getPublisher().isRemoved()));
      }
   }

   private void renderSubscriberRow(PubSubSubscriberStats subscriberStats)
   {
      if (subscriberStats.getSubscriber().getNumberOfReceivedMessages() > 0 || !hideInactiveTopics.get())
      {
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
         ImGui.text("%d".formatted(subscriberStats.getSubscriber().getNumberOfReceivedMessages()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(subscriberStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(subscriberStats.getReceiveFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getSubscriber().getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getSubscriber().getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(subscriberStats.getSubscriber().hasMatched()));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(subscriberStats.getSubscriber().isRemoved()));
      }
   }
}
