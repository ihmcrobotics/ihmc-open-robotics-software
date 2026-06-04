package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;

import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.TreeSet;

public class RDXROS2StatsPanel extends RDXPanel
{
   private static final CopyOnWriteArrayList<ROS2Node> trackedNodes = new CopyOnWriteArrayList<>();

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
   private final ImString filter = new ImString();

   private final HashMap<ROS2Publisher<?>, PubSubPublisherStats> publisherStatsMap = new HashMap<>();
   private final HashMap<ROS2Subscription<?>, PubSubSubscriberStats> subscriberStatsMap = new HashMap<>();
   private final TreeSet<ROS2Node> nodesSortedByName = new TreeSet<>(Comparator.comparing(ROS2Node::getName).thenComparingInt(System::identityHashCode));
   private final TreeSet<PubSubPublisherStats> publishersSortedBySize
         = new TreeSet<>(Comparator.<PubSubPublisherStats, Long>comparing(PubSubPublisherStats::getLargestMessageSize)
                                   .reversed()
                                   .thenComparingInt(System::identityHashCode));
   private final TreeSet<PubSubSubscriberStats> subscribersSortedBySize
         = new TreeSet<>(Comparator.<PubSubSubscriberStats, Long>comparing(PubSubSubscriberStats::getLargestMessageSize)
                                   .reversed()
                                   .thenComparingInt(System::identityHashCode));
   private final TreeSet<PubSubPublisherStats> publishersSortedByName
         = new TreeSet<>(Comparator.<PubSubPublisherStats, String>comparing(o -> o.getPublisher().getTopicName())
                                   .thenComparingInt(System::identityHashCode));
   private final TreeSet<PubSubSubscriberStats> subscribersSortedByName
         = new TreeSet<>(Comparator.<PubSubSubscriberStats, String>comparing(o -> o.getSubscription().getTopicName())
                                   .thenComparingInt(System::identityHashCode));

   public RDXROS2StatsPanel()
   {
      super("ROS 2 Stats", null, false, true);

      setRenderMethod(this::renderImGuiWidgets);
   }

   /**
    * Register a {@link ROS2Node} whose publishers and subscriptions should appear in this panel.
    * Closed nodes are removed automatically during rendering.
    */
   public static void registerNode(ROS2Node node)
   {
      if (node != null && !trackedNodes.contains(node))
      {
         trackedNodes.add(node);
      }
   }

   /**
    * Stop tracking a {@link ROS2Node} in this panel.
    */
   public static void unregisterNode(ROS2Node node)
   {
      trackedNodes.remove(node);
   }

   private void renderImGuiWidgets()
   {
      ImGui.inputText(labels.get("Search"), filter);

      numberOfMatchedSubscriptions = 0;
      numberOfPublications = 0;
      numberOfReceivedMessages = 0;
      largestMessageSize = 0;
      totalOutgoingBandwidth = 0.0;
      totalIncomingBandwidth = 0.0;

      nodesSortedByName.clear();
      publishersSortedBySize.clear();
      subscribersSortedBySize.clear();
      publishersSortedByName.clear();
      subscribersSortedByName.clear();

      for (ROS2Node node : collectTrackedNodes())
      {
         if (node.isClosed())
            continue;

         nodesSortedByName.add(node);

         List<ROS2Publisher<?>> publishers = node.getPublishers();
         synchronized (publishers)
         {
            for (ROS2Publisher<?> publisher : publishers)
            {
               PubSubPublisherStats publisherStats = publisherStatsMap.get(publisher);

               if (publisherStats == null)
               {
                  publisherStats = new PubSubPublisherStats(node, publisher);
                  publisherStatsMap.put(publisher, publisherStats);
               }

               if (sortByLargestMessageSize.get())
                  publishersSortedBySize.add(publisherStats);
               else
                  publishersSortedByName.add(publisherStats);
            }
         }

         List<ROS2Subscription<?>> subscriptions = node.getSubscriptions();
         synchronized (subscriptions)
         {
            for (ROS2Subscription<?> subscription : subscriptions)
            {
               PubSubSubscriberStats subscriberStats = subscriberStatsMap.get(subscription);

               if (subscriberStats == null)
               {
                  subscriberStats = new PubSubSubscriberStats(node, subscription);
                  subscriberStatsMap.put(subscription, subscriberStats);
               }

               if (sortByLargestMessageSize.get())
                  subscribersSortedBySize.add(subscriberStats);
               else
                  subscribersSortedByName.add(subscriberStats);
            }
         }
      }

      pruneClosedEndpoints();

      for (PubSubPublisherStats publisherStats : publisherStatsMap.values())
      {
         publisherStats.update();

         numberOfPublications += publisherStats.getSampleCount();
         totalOutgoingBandwidth += publisherStats.getBandwidth();

         if (publisherStats.getLargestMessageSize() > largestMessageSize)
            largestMessageSize = publisherStats.getLargestMessageSize();
      }

      for (PubSubSubscriberStats subscriberStats : subscriberStatsMap.values())
      {
         subscriberStats.update();

         if (subscriberStats.getSampleCount() > 0)
            numberOfMatchedSubscriptions += 1;

         numberOfReceivedMessages += subscriberStats.getSampleCount();
         totalIncomingBandwidth += subscriberStats.getBandwidth();

         if (subscriberStats.getLargestMessageSize() > largestMessageSize)
            largestMessageSize = subscriberStats.getLargestMessageSize();
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
         ImGui.tableSetupColumn(labels.get("Active Subscriptions"), ImGuiTableColumnFlags.WidthFixed);
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
         ImGui.text("%d".formatted(nodesSortedByName.size()));
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

      if (ImGui.beginTable(labels.get("Nodes"), 5, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Domain ID"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publishers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Subscribers"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Closed"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupScrollFreeze(0, 1);
         ImGui.tableHeadersRow();

         for (ROS2Node node : nodesSortedByName)
         {
            if (matchesFilter(node.getName()))
            {
               ImGui.tableNextRow();

               ImGui.tableNextColumn();
               ImGui.text(node.getName());
               ImGui.tableNextColumn();
               ImGui.text("%d".formatted(node.getDomainId()));
               ImGui.tableNextColumn();
               ImGui.text("%d".formatted(node.getPublishers().size()));
               ImGui.tableNextColumn();
               ImGui.text("%d".formatted(node.getSubscriptions().size()));
               ImGui.tableNextColumn();
               ImGui.text("%b".formatted(node.isClosed()));
            }
         }

         ImGui.endTable();
      }

      ImGuiTools.separatorText("Publishers");

      if (ImGui.beginTable(labels.get("Publishers"), 9, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Publications"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupColumn(labels.get("Closed"));
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

      if (ImGui.beginTable(labels.get("Subscribers"), 9, tableFlags))
      {
         ImGui.tableSetupColumn(labels.get("Node Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Topic Name"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Type"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Received"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Bandwidth"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Frequency"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Largest Message Size"), ImGuiTableColumnFlags.WidthFixed);
         ImGui.tableSetupColumn(labels.get("Current Message Size"));
         ImGui.tableSetupColumn(labels.get("Closed"));
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

   private static Set<ROS2Node> collectTrackedNodes()
   {
      Set<ROS2Node> nodes = new HashSet<>();
      nodes.addAll(ROS2Node.getActiveNodes());
      nodes.addAll(trackedNodes);
      return nodes;
   }

   private void pruneClosedEndpoints()
   {
      publisherStatsMap.entrySet().removeIf(entry -> entry.getKey().isClosed());
      subscriberStatsMap.entrySet().removeIf(entry -> entry.getKey().isClosed());
   }

   private void renderPublisherRow(PubSubPublisherStats publisherStats)
   {
      ROS2Node node = publisherStats.getNode();
      ROS2Publisher<?> publisher = publisherStats.getPublisher();
      String nodeName = node.getName();
      String topicName = publisher.getTopicName();
      String typeName = ROS2Message.getNameFromMessageClass(publisher.getTopicType());

      boolean show;
      if (!filter.isEmpty())
      {
         show = matchesFilter(nodeName, topicName, typeName);
      }
      else
      {
         show = publisherStats.getSampleCount() > 0 || !hideInactiveTopics.get();
      }

      if (show)
      {
         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text(nodeName);
         ImGui.tableNextColumn();
         ImGui.text(topicName);
         ImGui.tableNextColumn();
         ImGui.text(typeName);
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(publisherStats.getSampleCount()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(publisherStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(publisherStats.getPublishFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(publisherStats.getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(publisher.isClosed()));
      }
   }

   private void renderSubscriberRow(PubSubSubscriberStats subscriberStats)
   {
      ROS2Node node = subscriberStats.getNode();
      ROS2Subscription<?> subscription = subscriberStats.getSubscription();
      String nodeName = node.getName();
      String topicName = subscription.getTopicName();
      String typeName = ROS2Message.getNameFromMessageClass(subscription.getTopicType());

      boolean show;
      if (!filter.isEmpty())
      {
         show = matchesFilter(nodeName, topicName, typeName);
      }
      else
      {
         show = subscriberStats.getSampleCount() > 0 || !hideInactiveTopics.get();
      }

      if (show)
      {
         ImGui.tableNextRow();

         ImGui.tableNextColumn();
         ImGui.text(nodeName);
         ImGui.tableNextColumn();
         ImGui.text(topicName);
         ImGui.tableNextColumn();
         ImGui.text(typeName);
         ImGui.tableNextColumn();
         ImGui.text("%d".formatted(subscriberStats.getSampleCount()));
         ImGui.tableNextColumn();
         ImGui.text("%s/s".formatted(PubSubStatsTools.getHumanReadableBitSize(Math.round(subscriberStats.getBandwidth()))));
         ImGui.tableNextColumn();
         ImGui.text("%.0f Hz".formatted(subscriberStats.getReceiveFrequency()));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getLargestMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%s".formatted(PubSubStatsTools.getHumanReadableByteSize(subscriberStats.getCurrentMessageSize())));
         ImGui.tableNextColumn();
         ImGui.text("%b".formatted(subscription.isClosed()));
      }
   }

   private boolean matchesFilter(String... values)
   {
      if (filter.isEmpty())
         return true;

      String lowerFilter = filter.get().toLowerCase();
      for (String value : values)
      {
         if (value != null && value.toLowerCase().contains(lowerFilter))
            return true;
      }

      return false;
   }
}
