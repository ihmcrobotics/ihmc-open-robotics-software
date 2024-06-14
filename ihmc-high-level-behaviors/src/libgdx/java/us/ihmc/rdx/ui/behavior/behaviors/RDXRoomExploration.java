package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.room.RoomExplorationDefinition;
import us.ihmc.behaviors.room.RoomExplorationState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXRoomExploration extends RDXBehaviorTreeNode<RoomExplorationState, RoomExplorationDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXRoomExploration(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new RoomExplorationState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      ImGui.text("Room Exploration Node");
   }
}
