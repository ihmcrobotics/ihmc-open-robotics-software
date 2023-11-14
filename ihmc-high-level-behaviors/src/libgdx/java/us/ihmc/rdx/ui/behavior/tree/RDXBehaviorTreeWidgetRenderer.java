package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.rdx.imgui.*;

public class RDXBehaviorTreeWidgetRenderer
{
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();

   public void render(RDXBehaviorTree tree, BehaviorTreeModificationQueue modificationQueue, RDXBehaviorTreeNode<?, ?> node)
   {
      if (expandCollapseRenderer.render(node.getTreeWidgetExpanded()))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      ImGui.sameLine();
      node.renderTreeViewIconArea();

      ImGui.sameLine();
      node.renderNodeDescription();

      if (ImGui.beginPopup(node.getNodePopupID()))
      {
         // TODO

         ImGui.separator();

         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         if (ImGui.menuItem("Delete Node"))
         {
            modificationQueue.queueDestroySubtree(node);

            if (node.getParent() == null) // Root node
            {
               tree.setRootNode(null);
               tree.getBehaviorTreeState().freeze();
            }
         }
         ImGui.popStyleColor();

         ImGui.separator();
         if (ImGui.menuItem("Cancel"))
            ImGui.closeCurrentPopup();

         ImGui.endPopup();
      }

      if (node.getTreeWidgetExpanded())
      {
         float indentAmount = 10.0f;
         ImGui.indent(indentAmount);

         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            render(tree, modificationQueue, child);
         }

         ImGui.unindent(indentAmount);
      }
   }
}
