package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.commons.thread.Throttler;

/**
 * Top level class for the operator's behavior tree.
 */
public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private final ROS2BehaviorTreeState<RDXBehaviorTreeRootNode, BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition> ros2BehaviorTreeState;
   /** Reduce the communication update rate. */
   private final Throttler communicationThrottler = new Throttler().setFrequency(ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final ImGuiAveragedFrequencyText publishFrequencyText = new ImGuiAveragedFrequencyText();

   public RDXROS2BehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                              DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              ROS2PeerClockOffsetEstimator peerClockEstimator,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ReferenceFrameLibrary referenceFrameLibrary,
                              ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(treeFilesDirectory, robotModel, syncedRobot, peerClockEstimator, selectionCollisionModel, baseUI, panel3D, referenceFrameLibrary);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState<>(getBehaviorTreeState(), this::setRootNode, ros2);

      ros2BehaviorTreeState.getBehaviorTreeSubscription().registerMessageReceivedCallback(subscriptionFrequencyText::ping);
   }

   public void update()
   {
      boolean updateComms = communicationThrottler.run();
      if (updateComms)
      {
         ros2BehaviorTreeState.updateSubscription();
      }

      super.update();

      if (updateComms)
      {
         ros2BehaviorTreeState.updatePublication();
         publishFrequencyText.ping();
      }
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
      int numberOfLocalNodes = ros2BehaviorTreeState.getBehaviorTreeState().getNumberOfNodes();
      ImGui.text("Operator: %3d  Robot: %3d".formatted(numberOfLocalNodes, ros2BehaviorTreeState.getBehaviorTreeSubscription().getNumberOfOnRobotNodes()));

      ImGui.sameLine(ImGui.getWindowSizeX() - frequencyTextWidth - droppedTextWidth - rightMargin);
      ImGui.text("State:");

      ImGui.sameLine(ImGui.getWindowSizeX() - frequencyTextWidth - droppedTextWidth - rightMargin);
      subscriptionFrequencyText.render();

      ImGui.sameLine(ImGui.getWindowSizeX() - droppedTextWidth - rightMargin);
      ImGui.text("Dropped: %4d".formatted(ros2BehaviorTreeState.getBehaviorTreeSubscription().getMessageDropCount()));

      ImGui.endMenuBar();

      ImGui.text("CRDT#: Local: %d (%s)  Robot: %d  Out of order: %d"
                       .formatted(getBehaviorTreeState().getCRDTInfo().getUpdateNumber(),
                                  publishFrequencyText.getText(),
                                  ros2BehaviorTreeState.getBehaviorTreeSubscription().getPreviousSequenceID(),
                                  ros2BehaviorTreeState.getBehaviorTreeSubscription().getOutOfOrderCount()));

      super.renderImGuiWidgetsPost();
   }

   public void destroy()
   {
      ros2BehaviorTreeState.destroy();

      super.destroy();
   }
}
