package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.*;

public class RDXBehaviorTreeWidgetRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiHollowArrowRenderer hollowArrowRenderer = new ImGuiHollowArrowRenderer();
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private final ImVec2 padding = new ImVec2();
   private final ImVec2 labelSize = new ImVec2();

   public void render(RDXBehaviorTreeNode<?, ?> node)
   {
      String labelText = node.getDefinition().getDescription();

      padding.set(ImGui.getStyle().getFramePaddingX(), ImGui.getStyle().getFramePaddingY());
      ImGui.calcTextSize(labelSize, labelText);

      float frameHeight = labelSize.y + padding.y;

      int color = ImGui.getColorU32(ImGuiCol.Text);

      hollowArrowRenderer.render(0.7f, color);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() + padding.x);

      expandCollapseRenderer.render(node.getTreeWidgetExpanded(), 0.7f, color);

      if (expandCollapseRenderer.getIsHovered() && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() + padding.x);
      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text(labelText);
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
