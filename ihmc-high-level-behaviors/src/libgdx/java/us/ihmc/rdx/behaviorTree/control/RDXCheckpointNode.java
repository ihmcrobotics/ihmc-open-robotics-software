package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.CheckpointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.CheckpointNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiCheckpointNodeWidget;

public class RDXCheckpointNode extends RDXLeafNode<CheckpointNodeState, CheckpointNodeDefinition>
{
   private final ImGuiCheckpointNodeWidget checkpointNodeWidget = new ImGuiCheckpointNodeWidget();

   public RDXCheckpointNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new CheckpointNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      checkpointNodeWidget.render();

      renderRowEnd();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Checkpoint";
   }
}