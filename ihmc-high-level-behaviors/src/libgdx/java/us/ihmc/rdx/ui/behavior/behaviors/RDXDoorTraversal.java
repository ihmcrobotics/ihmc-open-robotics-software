package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.door.DoorTraversalDefinition;
import us.ihmc.behaviors.door.DoorTraversalState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXDoorTraversal extends RDXBehaviorTreeNode<DoorTraversalState, DoorTraversalDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DoorTraversalState state;
   private final ROS2SyncedRobotModel syncedRobot;

   public RDXDoorTraversal(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new DoorTraversalState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.syncedRobot = syncedRobot;

      getDefinition().setName("Door traversal");
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);

      if (state.getRetryingPullDoorNotification().poll())
         RDXBaseUI.pushNotification("Retrying pull door...");
   }

   public void updateActionSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {

         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(getDefinition().getClass().getSimpleName(), getState().getID()));

      if (state.isTreeStructureValid())
      {
         ImGui.text("Pull screw primitive node: Executing: %b".formatted(state.getPullScrewPrimitiveAction().getIsExecuting()));
      }
      else
      {
         ImGui.textColored(ImGuiTools.DARK_RED, "Expected node(s) could not be found by name.");
      }

      super.renderNodeSettingsWidgets();
   }
}