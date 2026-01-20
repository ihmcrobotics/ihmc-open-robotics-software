package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
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

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      // Validate state in case something earlier in this UI tick messed with things.
      // This happens with the Undo non-topological changes button.
      state.validateDefinition(rootNode.getState().getOrderedLeaves());

      if (ImGui.beginCombo(labels.get("Goto"), definition.getNodeToGotoName()))
      {
         if (ImGui.selectable(labels.get("Next"), definition.getGotoNextNode()))
         {
            definition.setGotoNextNode();
         }

         for (LeafNodeState<?> leafNode : rootNode.getState().getOrderedLeaves())
         {
            if (leafNode != state) // Exclude self
            {
               if (ImGui.selectable(leafNode.getDefinition().getName(), definition.getNodeToGotoID() == leafNode.getID()))
               {
                  definition.setNodeToGoto(leafNode.getID(), leafNode.getDefinition().getName());
               }
            }
         }

         ImGui.endCombo();
      }
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Goto Node";
   }
}