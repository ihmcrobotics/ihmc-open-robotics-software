package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceDefinition;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.ui.widgets.ImGuiSequenceIconWidget;

public class RDXActionSequence extends RDXBehaviorTreeNode<ActionSequenceState, ActionSequenceDefinition>
{
   private final ImGuiSequenceIconWidget sequenceIconWidget = new ImGuiSequenceIconWidget();

   public RDXActionSequence(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new ActionSequenceState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();

      sequenceIconWidget.render();

      ImGui.sameLine();
      super.renderEditableName();
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));

      super.renderNodeSettingsWidgets();
   }
}