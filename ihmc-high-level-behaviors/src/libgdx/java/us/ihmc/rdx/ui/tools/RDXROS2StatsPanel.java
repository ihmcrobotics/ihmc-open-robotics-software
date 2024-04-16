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

   private final HashMap<Publisher, ROS2PublisherStats> publisherStatsMap = new HashMap<>();
   private final HashMap<Subscriber<?>, ROS2SubscriberStats> subscriberStatsMap = new HashMap<>();
   private final TreeSet<Participant> participantsSortedByName = new TreeSet<>(Comparator.<Participant, String>comparing(o -> o.getAttributes().getName())
                                                                                         .thenComparingInt(Object::hashCode));
   private final TreeSet<ROS2PublisherStats> publishersSortedBySize
         = new TreeSet<>(Comparator.<ROS2PublisherStats, Long>comparing(o -> o.getPublisher().getLargestMessageSize())
                                   .reversed()
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<ROS2SubscriberStats> subscribersSortedBySize
         = new TreeSet<>(Comparator.<ROS2SubscriberStats, Long>comparing(o -> o.getSubscriber().getLargestMessageSize())
                                   .reversed()
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<ROS2PublisherStats> publishersSortedByName
         = new TreeSet<>(Comparator.<ROS2PublisherStats, String>comparing(o -> o.getPublisher().getAttributes().getHumanReadableTopicName())
                                   .thenComparingInt(Object::hashCode));
   private final TreeSet<ROS2SubscriberStats> subscribersSortedByName
         = new TreeSet<>(Comparator.<ROS2SubscriberStats, String>comparing(o -> o.getSubscriber().getAttributes().getHumanReadableTopicName())
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
                  ROS2PublisherStats publisherStats = publisherStatsMap.get(publisher);

                  if (publisherStats == null)
                  {
                     publisherStats = new ROS2PublisherStats(participant, publisher);
                     publisherStatsMap.put(publisher, publisherStats);
                  }

                  if (sortByLargestMessageSize.get())
                     publishersSortedBySize.add(publisherStats);
                  else
                     publishersSortedByName.add(publisherStats);

                  publisherStats.update();

                  numberOfPublications += publisher.getNumberOfPublications();
                  totalOutgoingBandwidth += publisherStats.getBandwidth();

                  if (publisher.getLargestMessageSize() > largestMessageSize)
                     largestMessageSize = publisher.getLargestMessageSize();
               }
            }

            List<Subscriber<?>> subscribers = participant.getAllSubscribersForStatistics();
            synchronized (subscribers)
            {
               for (Subscriber<?> subscriber : subscribers)
               {
                  ROS2SubscriberStats subscriberStats = subscriberStatsMap.get(subscriber);

                  if (subscriberStats == null)
                  {
                     subscriberStats = new ROS2SubscriberStats(participant, subscriber);
                     subscriberStatsMap.put(subscriber, subscriberStats);
                  }

                  if (sortByLargestMessageSize.get())
                     subscribersSortedBySize.add(subscriberStats);
                  else
                     subscribersSortedByName.add(subscriberStats);

                  numberOfMatchedSubscriptions += subscriber.hasMatched() ? 1 : 0;
                  numberOfReceivedMessages += subscriber.getNumberOfReceivedMessages();
                  totalIncomingBandwidth += subscriberStats.getBandwidth();

                  if (subscriber.getLargestMessageSize() > largestMessageSize)
                     largestMessageSize = subscriber.getLargestMessageSize();
               }
            }
         }
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
         ImGui.text(PubSubStatsTools.getHumanReadableByteSize(largestMessageSize));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(totalOutgoingBandwidth))));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(totalIncomingBandwidth))));

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
            for (ROS2PublisherStats publisherStats : publishersSortedBySize)
               renderPublisherRow(publisherStats);
         else
            for (ROS2PublisherStats publisherStats : publishersSortedByName)
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
            for (ROS2SubscriberStats subscriberStats : subscribersSortedBySize)
               renderSubscriberRow(subscriberStats);
         else
            for (ROS2SubscriberStats subscriberStats : subscribersSortedByName)
               renderSubscriberRow(subscriberStats);

         ImGui.endTable();
      }
   }

   private void renderPublisherRow(ROS2PublisherStats publisherStats)
   {
      if (publisherStats.getPublisher().getNumberOfPublications() > 0 || !hideInactiveTopics.get())
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

   private void renderSubscriberRow(ROS2SubscriberStats subscriberStats)
   {
      if (subscriberStats.getSubscriber().getNumberOfReceivedMessages() > 0 || !hideInactiveTopics.get())
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
