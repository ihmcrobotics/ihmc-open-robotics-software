package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTree;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.commons.thread.Throttler;

/**
 * Top level class for the operator's behavior tree.
 */
public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private final ROS2BehaviorTree<RDXBehaviorTreeNode<?, ?>> ros2BehaviorTree;
   /** Reduce the communication update rate. */
   private final Throttler communicationThrottler = new Throttler().setFrequency(ROS2BehaviorTree.SYNC_FREQUENCY);
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImGuiAveragedFrequencyText publishFrequencyText = new ImGuiAveragedFrequencyText();

   public RDXROS2BehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                              ROS2SyncedRobotModel syncedRobot,
                              ROS2PeerClockOffsetEstimator peerClockEstimator,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(treeFilesDirectory, syncedRobot, peerClockEstimator, selectionCollisionModel, baseUI, panel3D);

      ros2BehaviorTree = new ROS2BehaviorTree<>((BehaviorTree) this, ros2);

      ros2BehaviorTree.getBehaviorTreeSubscription().registerMessageReceivedCallback(subscriptionFrequencyText::ping);
   }

   public void update()
   {
      if (communicationThrottler.run())
      {
         // Must publish first to get newly modified data published
         // This is because data gets modified after the update in the
         // ImGui rendering thread.
         ros2BehaviorTree.updatePublication();
         publishFrequencyText.ping();
         ros2BehaviorTree.updateSubscription();
      }

      super.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPre();

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

   public void destroy()
   {
      ros2BehaviorTree.destroy();

      super.destroy();
   }
}
