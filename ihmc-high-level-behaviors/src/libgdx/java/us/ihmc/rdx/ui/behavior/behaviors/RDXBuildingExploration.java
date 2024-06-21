package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationDefinition;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXBuildingExploration extends RDXBehaviorTreeNode<BuildingExplorationState, BuildingExplorationDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXBuildingExploration(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new BuildingExplorationState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;

      getDefinition().setName("Building Exploration");
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      ImGui.text("Building Exploration Node");
   }
}
