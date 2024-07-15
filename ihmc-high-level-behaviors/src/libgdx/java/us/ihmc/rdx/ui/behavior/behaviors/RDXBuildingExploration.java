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

      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_APPROACH_RIGHT_PUSH, state.getSetStaticForApproachAction());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_GRASP_RIGHT_PUSH, state.getSetStaticForGraspAction());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_APPROACH_PUSH, state.getSetStaticForApproachActionPush());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_GRASP_PUSH, state.getSetStaticForGraspActionPush());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_APPROACH_PULL, state.getSetStaticForApproachActionPull());
      renderNodePresenceStatus(BuildingExplorationState.SET_STATIC_FOR_GRASP_PULL, state.getSetStaticForGraspActionPull());

      renderNodePresenceStatus(BuildingExplorationState.END_FIRST_DOOR, state.getEndFirstDoorAction());
      renderNodePresenceStatus(BuildingExplorationState.START_SCAN, state.getStartScanAction());
      renderNodePresenceStatus(BuildingExplorationState.END_SCAN, state.getEndScanAction());
      renderNodePresenceStatus(BuildingExplorationState.START_PULL_DOOR, state.getStartPullDoorAction());
      renderNodePresenceStatus(BuildingExplorationState.END_PULL_DOOR, state.getEndPullDoorAction());
      renderNodePresenceStatus(BuildingExplorationState.START_PUSH_DOOR, state.getStartPushDoorAction());
      renderNodePresenceStatus(BuildingExplorationState.END_PUSH_DOOR, state.getEndPushDoorAction());
      renderNodePresenceStatus(BuildingExplorationState.START_TRASHCAN, state.getStartTrashCanAction());
      renderNodePresenceStatus(BuildingExplorationState.END_TRASHCAN, state.getEndTrashCanAction());
      renderNodePresenceStatus(BuildingExplorationState.START_COUCH, state.getStartCouchAction());
      renderNodePresenceStatus(BuildingExplorationState.END_COUCH, state.getEndCouchAction());
      renderNodePresenceStatus(BuildingExplorationState.START_TABLE_LEFT, state.getStartTableLeftAction());
      renderNodePresenceStatus(BuildingExplorationState.END_TABLE_LEFT, state.getEndTableLeftAction());
      renderNodePresenceStatus(BuildingExplorationState.START_TABLE_RIGHT, state.getStartTableRightAction());
      renderNodePresenceStatus(BuildingExplorationState.END_TABLE_RIGHT, state.getEndTableRightAction());
      renderNodePresenceStatus(BuildingExplorationState.END_TABLE_RIGHT, state.getEndTableRightAction());
      renderNodePresenceStatus(BuildingExplorationState.START_SALUTE, state.getStartSaluteAction());

      renderNodePresenceStatus(BuildingExplorationState.WALK_DOOR_A, state.getWalkDoorAAction());
      renderNodePresenceStatus(BuildingExplorationState.WALK_DOOR_B, state.getWalkDoorBAction());
      renderNodePresenceStatus(BuildingExplorationState.TURN_DOOR_A, state.getTurnDoorAAction());
      renderNodePresenceStatus(BuildingExplorationState.TURN_DOOR_B, state.getTurnDoorBAction());
      renderNodePresenceStatus(BuildingExplorationState.END_WALK_COUCH, state.getWalkCouchAction());
      renderNodePresenceStatus(BuildingExplorationState.END_DEMO, state.getEndDemoAction());

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
