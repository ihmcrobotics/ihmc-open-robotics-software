package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.GotoNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.behaviorTree.RDXLeafNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.widgets.ImGuiGotoNodeWidget;

public class RDXGotoNode extends RDXLeafNode<GotoNodeState, GotoNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiGotoNodeWidget gotoNodeWidget = new ImGuiGotoNodeWidget();

   public RDXGotoNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new GotoNodeState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      gotoNodeWidget.render();
      ImGui.sameLine();
      ImGui.textDisabled("%s".formatted(definition.getNodeToGotoName()));

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      // Validate state in case something earlier in this UI tick messed with things.
      // This happens with the Undo non-topological changes button.
      state.validateDefinition(rootNode.getState().getOrderedNodes());

      if (ImGui.beginCombo(labels.get("Goto"), definition.getNodeToGotoName()))
      {
         if (ImGui.selectable(labels.get("Next"), definition.getGotoNextNode()))
            definition.setGotoNextNode();

         for (BehaviorTreeNodeState<?> node : rootNode.getState().getOrderedNodes())
            if (node != state) // Exclude self
               if (ImGui.selectable(node.getDefinition().getName(), definition.getNodeToGotoID() == node.getID()))
                  definition.setNodeToGoto(node.getID(), node.getDefinition().getName());

         ImGui.endCombo();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Goto Node";
   }
}