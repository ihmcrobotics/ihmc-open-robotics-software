package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.ai2r.AI2RNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXAI2RNode extends RDXBehaviorTreeNode<AI2RNodeState, AI2RNodeDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXAI2RNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new AI2RNodeState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;

      getDefinition().setName("AI2R Node");
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      super.renderNodeSettingsWidgets();

      ImGui.text("AI2R Node");
   }
}
