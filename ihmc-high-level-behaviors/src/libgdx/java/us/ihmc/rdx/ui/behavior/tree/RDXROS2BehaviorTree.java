package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeState;
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

/**
 * Top level class for the operator's behavior tree.
 */
public class RDXROS2BehaviorTree extends RDXBehaviorTree
{
   private final ROS2BehaviorTreeState ros2BehaviorTreeState;
   /** Reduce the communication update rate. */
   private final Throttler communicationThrottler = new Throttler().setFrequency(ROS2BehaviorTreeState.SYNC_FREQUENCY);
   private final RDXPanel panel = new RDXPanel("Behavior Tree", this::renderImGuiWidgets, false, true);
   private final ImGuiAveragedFrequencyText subscriptionFrequencyText = new ImGuiAveragedFrequencyText();
   private final transient ImVec2 statusTextSize = new ImVec2();

   public RDXROS2BehaviorTree(WorkspaceResourceDirectory treeFilesDirectory,
                              DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              RobotCollisionModel selectionCollisionModel,
                              RDXBaseUI baseUI,
                              RDX3DPanel panel3D,
                              ReferenceFrameLibrary referenceFrameLibrary,
                              FootstepPlannerParametersBasics footstepPlannerParametersBasics,
                              ROS2ControllerPublishSubscribeAPI ros2)
   {
      super(treeFilesDirectory,
            robotModel,
            syncedRobot,
            selectionCollisionModel,
            baseUI,
            panel3D,
            referenceFrameLibrary,
            footstepPlannerParametersBasics);

      ros2BehaviorTreeState = new ROS2BehaviorTreeState(getBehaviorTreeState(), this::setRootNode, ros2);

      ros2BehaviorTreeState.getBehaviorTreeSubscription().registerMessageReceivedCallback(subscriptionFrequencyText::ping);
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

      if (updateComms)
         ros2BehaviorTreeState.updatePublication();
   }

   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgetsPre();

      // Prevent jumping around when it changes
      ImGui.calcTextSize(statusTextSize, "Operator: 999 Robot: 999 ");
      float nodeCountsTextWidth = statusTextSize.x;
      ImGui.calcTextSize(statusTextSize, "State: Frozen (999)");
      float frozenStatusTextWidth = statusTextSize.x;
      ImGui.calcTextSize(statusTextSize, "999 Hz");
      float frequencyTextWidth = statusTextSize.x;
      float rightMargin = 20.0f;

      ImGui.sameLine(ImGui.getWindowSizeX() - nodeCountsTextWidth - frozenStatusTextWidth - frequencyTextWidth - rightMargin);
      int numberOfLocalNodes = ros2BehaviorTreeState.getBehaviorTreeState().getNumberOfNodes();
      ImGui.text("Operator: %3d  Robot: %3d".formatted(numberOfLocalNodes, ros2BehaviorTreeState.getBehaviorTreeSubscription().getNumberOfOnRobotNodes()));

      ImGui.sameLine(ImGui.getWindowSizeX() - frozenStatusTextWidth - frequencyTextWidth - rightMargin);
      int numberOfFrozenNodes = ros2BehaviorTreeState.getBehaviorTreeState().getNumberOfFrozenNodes();
      ImGui.text("State:");
      ImGui.sameLine();
      if (ros2BehaviorTreeState.getBehaviorTreeState().isFrozen() || numberOfFrozenNodes > 0)
         ImGui.textColored(ImGuiTools.LIGHT_BLUE, "Frozen (%d)".formatted(numberOfFrozenNodes));
      else
         ImGui.text("Normal");

      ImGui.sameLine(ImGui.getWindowSizeX() - frequencyTextWidth - rightMargin);
      subscriptionFrequencyText.render();

      super.renderImGuiWidgetsPost();
   }

   public void destroy()
   {
      ros2BehaviorTreeState.destroy();

      super.destroy();
   }
}
