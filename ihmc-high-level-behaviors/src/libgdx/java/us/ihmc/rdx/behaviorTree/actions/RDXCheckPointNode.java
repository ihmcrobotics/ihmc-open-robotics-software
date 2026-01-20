package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.CheckPointNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiCheckpointNodeWidget;

public class RDXCheckPointNode extends RDXLeafNode<CheckPointNodeState, CheckPointNodeDefinition>
{
   private final ImGuiCheckpointNodeWidget checkpointNodeWidget = new ImGuiCheckpointNodeWidget();

   public RDXCheckPointNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new CheckPointNodeState(id, rootNode.getState()), rootNode);
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
      return "Check point";
   }
}