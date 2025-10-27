package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.FallbackNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.ui.widgets.ImGuiFallbackWidget;

public class RDXFallbackNode extends RDXBehaviorTreeNode<FallbackNodeState, FallbackNodeDefinition>
{
   private final ImGuiFallbackWidget fallbackWidget = new ImGuiFallbackWidget();

   public RDXFallbackNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new FallbackNodeState(id, rootNode.getState()), rootNode);
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

      fallbackWidget.render();

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