package us.ihmc.rdx.ui.behavior.tree;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.Throttler;

public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;
   /** Reduce the communication update rate. */
   private final Throttler communicationThrottler = new Throttler().setFrequency(30.0);
   private final RDXPanel panel = new RDXPanel("Behavior Tree", this::renderImGuiWidgets, false, true);
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();

   public RDXROS2BehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                              DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ReferenceFrameLibrary referenceFrameLibrary,
                              FootstepPlannerParametersBasics footstepPlannerParametersBasics, ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(treeFilesDirectory,
            robotModel,
            syncedRobot,
            selectionCollisionModel,
            baseUI,
            panel3D,
            referenceFrameLibrary,
            footstepPlannerParametersBasics);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState(getBehaviorTreeState(), this::setRootNode, ros2, ROS2ActorDesignation.OPERATOR);

      ros2BehaviorTreeState.getBehaviorTreeSubscription().getBehaviorTreeSubscription().addCallback(message -> subscriptionFrequencyText.ping());
   }

   public void createAndSetupDefault(RDXBaseUI baseUI)
   {
      baseUI.getImGuiPanelManager().addPanel(panel);
      super.createAndSetupDefault(baseUI);
   }

   public void update()
   {
      boolean updateComms = communicationThrottler.run();
      if (updateComms)
         ros2BehaviorTreeState.updateSubscription();

      super.update();

      if (getRootNode() != null && updateComms)
         ros2BehaviorTreeState.updatePublication();
   }

   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPre();

      int numberOfLocalNodes = ros2BehaviorTreeState.getBehaviorTreeState().getNumberOfNodes();
      BehaviorTreeStateMessage latestBehaviorTreeMessage = ros2BehaviorTreeState.getBehaviorTreeSubscription().getLatestBehaviorTreeMessage();
      int numberOfOnRobotNodes = latestBehaviorTreeMessage == null ? 0 : latestBehaviorTreeMessage.getBehaviorTreeIndices().size();
      ImGui.text("Nodes: UI: %d   On-Robot: %d   State: ".formatted(numberOfLocalNodes, numberOfOnRobotNodes));
      ImGui.sameLine();
      if (ros2BehaviorTreeState.getBehaviorTreeState().getLocalTreeFrozen())
         ImGui.textColored(ImGuiTools.LIGHT_BLUE, "Frozen");
      else
         ImGui.text("Normal");
      ImGui.pushItemWidth(100.0f);
      ImGui.sameLine();
      subscriptionFrequencyText.render();
      ImGui.popItemWidth();

      super.renderImGuiWidgetsPost();
   }

   public void destroy()
   {
      ros2BehaviorTreeState.destroy();

      super.destroy();
   }
}
