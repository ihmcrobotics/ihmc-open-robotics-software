package us.ihmc.rdx.ui.behavior.behaviors;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionDefinition;
import us.ihmc.behaviors.behaviorTree.trashCan.TrashCanInteractionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import javax.annotation.Nullable;

public class RDXTrashCanInteraction extends RDXBehaviorTreeNode<TrashCanInteractionState, TrashCanInteractionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final TrashCanInteractionState state;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImGuiInputText obstructNodeName;

   public RDXTrashCanInteraction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot)
   {
      super(new TrashCanInteractionState(id, crdtInfo, saveFileDirectory));

      state = getState();
      this.syncedRobot = syncedRobot;

      getDefinition().setName("Trash can interaction");
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      obstructNodeName = new ImGuiInputText("Obstructed Node: ");
      obstructNodeName.addWidgetAligner(widgetAligner);
   }

   @Override
   public void update()
   {
      super.update();

      updateActionSubtree(this);
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

      renderNodePresenceStatus(TrashCanInteractionState.COMPUTE_STANCE, state.getComputeStanceAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACHING_FRONT, state.getApproachingFrontAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACHING_LEFT, state.getApproachingLeftAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACHING_RIGHT, state.getApproachingRightAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACH_FRONT, state.getApproachFrontAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACH_LEFT, state.getApproachLeftAction());
      renderNodePresenceStatus(TrashCanInteractionState.APPROACH_RIGHT, state.getApproachRightAction());
      renderNodePresenceStatus(TrashCanInteractionState.LEFT_FOOT_DOWN, state.getSetLeftFootDownAction());
      renderNodePresenceStatus(TrashCanInteractionState.END, state.getEndAction());

      ImGui.text("Trash can interaction: ");
      ImGui.sameLine();
      if (state.areLogicNodesPresent())
         ImGui.textColored(ImGuiTools.DARK_GREEN, "ENABLED");
      else
         ImGui.textColored(ImGuiTools.DARK_RED, "DISABLED");

      obstructNodeName.render();
      ImGui.text("Approach stance: " + state.getStance().getValue().toString());

      super.renderNodeSettingsWidgets();
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
}
