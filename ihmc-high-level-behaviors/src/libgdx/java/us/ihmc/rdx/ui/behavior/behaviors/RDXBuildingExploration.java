package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;
import java.util.List;

public class RDXBuildingExploration extends RDXBehaviorTreeNode<BuildingExplorationState, BuildingExplorationDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final BuildingExplorationState state;

   public RDXBuildingExploration(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new BuildingExplorationState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;
      state = getState();

      getDefinition().setName("Building Exploration");
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_APPROACH, state.getSetStaticForApproachActions());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_GRASP, state.getSetStaticForGraspActions());

      ImGui.text("Building Exploration Node");
   }

   private void renderNodePresenceStatus(String expectedName, @Nullable BehaviorTreeNodeState<?> node)
   {
      ImGui.text("%s: ".formatted(expectedName));
      ImGui.sameLine();
      if (node == null)
         ImGui.textColored(ImGuiTools.DARK_RED, "MISSING");
      else
         ImGui.textColored(ImGuiTools.DARK_GREEN, "FOUND");
   }

   private void renderNodePresenceStatus(String expectedName, List<WaitDurationActionState> nodes)
   {
      ImGui.text("%s: ".formatted(expectedName));
      ImGui.sameLine();
      if (nodes.isEmpty())
         ImGui.textColored(ImGuiTools.DARK_RED, "MISSING");
      else
         ImGui.textColored(ImGuiTools.DARK_GREEN, "FOUND %d".formatted(nodes.size()));
   }
}
