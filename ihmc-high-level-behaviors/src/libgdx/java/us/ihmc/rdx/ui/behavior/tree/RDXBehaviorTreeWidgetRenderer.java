package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import us.ihmc.rdx.imgui.*;

public class RDXBehaviorTreeWidgetRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiHollowArrowRenderer hollowArrowRenderer = new ImGuiHollowArrowRenderer();
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();

   public void render(RDXBehaviorTreeNode<?, ?> node)
   {
      if (expandCollapseRenderer.render(node.getTreeWidgetExpanded()))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      node.renderTreeViewIconArea();

      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text(node.getDefinition().getDescription());
      ImGui.popFont();

      if (node.getTreeWidgetExpanded())
      {
         float indentAmount = 10.0f;
         ImGui.indent(indentAmount);

         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            render(child);
         }

         ImGui.unindent(indentAmount);
      }
   }
}
