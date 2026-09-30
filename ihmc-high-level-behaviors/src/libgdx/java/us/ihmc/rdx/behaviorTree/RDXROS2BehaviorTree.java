package us.ihmc.rdx.behaviorTree;

import behavior_msgs.BehaviorTreeLeafStatusMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTree;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Top level class for the operator's behavior tree.
 */
public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private static final int MAX_STATUS_LEAVES = 120;

   private final ROS2BehaviorTree<RDXBehaviorTreeNode<?, ?>> ros2BehaviorTree;
   private final ROS2ControllerHelper ros2ControllerHelper;
   /** Reduce the communication update rate. */
   private final Throttler communicationThrottler = new Throttler().setFrequency(ROS2BehaviorTree.SYNC_FREQUENCY);
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImGuiAveragedFrequencyText publishFrequencyText = new ImGuiAveragedFrequencyText();
   /** Unchecked keeps the editor CRDT. Checked watches leaf flags and does not publish or apply the CRDT. */
   private final ImBoolean monitorMode = new ImBoolean(false);
   private ROS2Subscription<BehaviorTreeLeafStatusMessage> leafStatusSubscription;
   private final ConcurrentLinkedQueue<LeafStatusSample> leafStatusSamples = new ConcurrentLinkedQueue<>();
   private final ArrayList<LeafStatusSample> pendingStatusSamples = new ArrayList<>();
   private String monitoredTreeFile = "";
   private final double[] elapsedBase = new double[MAX_STATUS_LEAVES];
   private final long[] elapsedReceiptNanos = new long[MAX_STATUS_LEAVES];
   private final boolean[] trackingElapsed = new boolean[MAX_STATUS_LEAVES];

   public RDXROS2BehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                              ROS2SyncedRobotModel syncedRobot,
                              ROS2PeerClockOffsetEstimator peerClockEstimator,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ROS2ControllerHelper ros2)
   {
      super(treeFilesDirectory, syncedRobot, peerClockEstimator, selectionCollisionModel, baseUI, panel3D);

      ros2ControllerHelper = ros2;
      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2.getROS2Node());

      ros2BehaviorTree.getBehaviorTreeSubscription().registerMessageReceivedCallback(subscriptionFrequencyText::ping);
   }

   public void update()
   {
      if (communicationThrottler.run())
      {
         if (monitorMode.get())
         {
            ensureLeafStatusSubscription();
         }
         else
         {
            destroyLeafStatusSubscription();
            // Must publish first to get newly modified data published
            // This is because data gets modified after the update in the
            // ImGui rendering thread.
            ros2BehaviorTree.updatePublication();
            publishFrequencyText.ping();
            ros2BehaviorTree.updateSubscription();
         }
      }

      if (monitorMode.get())
         loadIncomingMonitorTrees();

      super.update();

      if (monitorMode.get())
      {
         applyIncomingMonitorStatus();
         advanceMonitorElapsed();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPre();

      ImGui.sameLine();
      ImGui.checkbox("Monitor", monitorMode);

      // Prevent jumping around when it changes
      float nodeCountsTextWidth = ImGuiTools.calcTextSizeX("Operator: 000 Robot: 000 ");
      float frequencyTextWidth = ImGuiTools.calcTextSizeX("000 Hz ");
      float droppedTextWidth = ImGuiTools.calcTextSizeX("Dropped: 0000");
      float rightMargin = 20.0f;

      ImGui.sameLine(ImGui.getWindowSizeX() - nodeCountsTextWidth - frequencyTextWidth - droppedTextWidth - rightMargin);
      ImGui.text("Operator: %3d  Robot: %3d".formatted(numberOfNodes, ros2BehaviorTree.getBehaviorTreeSubscription().getNumberOfOnRobotNodes()));


      ImGui.sameLine(ImGui.getWindowSizeX() - frequencyTextWidth - droppedTextWidth - rightMargin);
      subscriptionFrequencyText.render();

      ImGui.sameLine(ImGui.getWindowSizeX() - droppedTextWidth - rightMargin);
      ImGui.text("Dropped: %4d".formatted(ros2BehaviorTree.getBehaviorTreeSubscription().getMessageDropCount()));

      ImGui.endMenuBar();

      ImGui.text("CRDT#: Local: %d (%s)  Robot: %d  Out of order: %d"
                       .formatted(getCRDTInfo().getUpdateNumber(),
                                  publishFrequencyText.getText(),
                                  ros2BehaviorTree.getBehaviorTreeSubscription().getPreviousSequenceID(),
                                  ros2BehaviorTree.getBehaviorTreeSubscription().getOutOfOrderCount()));

      super.renderImGuiWidgetsPost();
   }

   @Override
   public void destroy()
   {
      destroyLeafStatusSubscription();
      ros2BehaviorTree.destroy();

      super.destroy();
   }

   public ROS2ControllerHelper getROS2ControllerHelper()
   {
      return ros2ControllerHelper;
   }

   public ROS2Node getROS2Node()
   {
      return ros2ControllerHelper.getROS2Node();
   }

   private void ensureLeafStatusSubscription()
   {
      if (leafStatusSubscription != null)
         return;

      leafStatusSubscription = getROS2Node().createSubscription(AutonomyAPI.BEHAVIOR_TREE_LEAF_STATUS, reader ->
      {
         BehaviorTreeLeafStatusMessage message = reader.read();
         if (message == null)
            return;
         // The DDS buffer is reused on the next sample, so copy before returning.
         leafStatusSamples.add(new LeafStatusSample(message));
      });
   }

   private void destroyLeafStatusSubscription()
   {
      if (leafStatusSubscription == null)
         return;

      getROS2Node().destroySubscription(leafStatusSubscription);
      leafStatusSubscription = null;
      leafStatusSamples.clear();
      pendingStatusSamples.clear();
      monitoredTreeFile = "";
      Arrays.fill(trackingElapsed, false);
   }

   /** Loads a snapshot's file before the tree update so leaf indices exist when flags are applied. */
   private void loadIncomingMonitorTrees()
   {
      LeafStatusSample sample;
      while ((sample = leafStatusSamples.poll()) != null)
      {
         String treeFile = sample.message.getTreeFileAsString();
         if (sample.message.getSnapshot() && !treeFile.isBlank() && !treeFile.equals(monitoredTreeFile))
         {
            if (loadBehavior(treeFile))
               monitoredTreeFile = treeFile;
            else
               sample.applyFlags = false;
         }
         pendingStatusSamples.add(sample);
      }
   }

   private void applyIncomingMonitorStatus()
   {
      if (getRootNode() == null)
      {
         pendingStatusSamples.clear();
         return;
      }

      List<LeafNodeState<?>> leaves = getRootNode().getState().getOrderedLeaves();
      for (int s = 0; s < pendingStatusSamples.size(); s++)
      {
         LeafStatusSample sample = pendingStatusSamples.get(s);
         if (!sample.applyFlags)
            continue;
         if (sample.message.getSnapshot())
            Arrays.fill(trackingElapsed, false);

         int count = sample.message.getLeafIndex().size();
         for (int i = 0; i < count; i++)
         {
            int index = sample.message.getLeafIndex().get(i);
            if (index < 0 || index >= leaves.size() || index >= MAX_STATUS_LEAVES)
               continue;

            LeafNodeState<?> leaf = leaves.get(index);
            if (leaf.getLeafIndex() != index)
               continue;

            boolean executing = sample.message.getIsExecuting().get(i);
            leaf.applyMonitorStatus(sample.message.getIsActive().get(i),
                                    sample.message.getIsNextForExecution().get(i),
                                    sample.message.getCanExecute().get(i),
                                    executing,
                                    sample.message.getFailed().get(i));
            trackingElapsed[index] = false;
            double elapsed = sample.message.getElapsedSeconds().get(i);
            if (executing && leaf instanceof ActionNodeState<?> action && Double.isFinite(elapsed))
            {
               action.applyMonitorElapsed(elapsed);
               elapsedBase[index] = elapsed;
               elapsedReceiptNanos[index] = sample.receiptNanos;
               trackingElapsed[index] = true;
            }
         }
      }
      pendingStatusSamples.clear();
   }

   /** Moves the timeline between the once-a-second elapsed samples while the leaf is still executing. */
   private void advanceMonitorElapsed()
   {
      if (getRootNode() == null)
         return;

      List<LeafNodeState<?>> leaves = getRootNode().getState().getOrderedLeaves();
      long now = System.nanoTime();
      for (int index = 0; index < leaves.size() && index < MAX_STATUS_LEAVES; index++)
      {
         if (!trackingElapsed[index])
            continue;

         LeafNodeState<?> leaf = leaves.get(index);
         if (leaf.getLeafIndex() != index || !leaf.getIsExecuting() || !(leaf instanceof ActionNodeState<?> action))
         {
            trackingElapsed[index] = false;
            continue;
         }

         double seconds = (now - elapsedReceiptNanos[index]) * 1.0e-9;
         action.applyMonitorElapsed(elapsedBase[index] + seconds);
      }
   }

   /** Copy of one DDS sample. The callback's message is reused on the next read. */
   private static final class LeafStatusSample
   {
      private boolean applyFlags = true;
      private final BehaviorTreeLeafStatusMessage message;
      private final long receiptNanos;

      private LeafStatusSample(BehaviorTreeLeafStatusMessage message)
      {
         this.message = new BehaviorTreeLeafStatusMessage(message);
         receiptNanos = System.nanoTime();
      }
   }
}
