package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiDir;
import imgui.internal.ImRect;
import us.ihmc.rdx.imgui.ImGuiArrowRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXBehaviorTreeWidgetRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiArrowRenderer arrowRenderer = new ImGuiArrowRenderer();
   private final ImVec2 padding = new ImVec2();
   private final ImVec2 labelSize = new ImVec2();
   private final ImRect interactionBoundingBox = new ImRect();

   public void render(RDXBehaviorTreeNode<?, ?> node)
   {
      String labelText = node.getDefinition().getDescription();

      padding.set(ImGui.getStyle().getFramePaddingX(), ImGui.getStyle().getFramePaddingY());
      ImGui.calcTextSize(labelSize, labelText);

      float frameHeight = labelSize.y + padding.y;

      interactionBoundingBox.min.x = ImGui.getCursorPosX();
      interactionBoundingBox.min.y = ImGui.getCursorPosY();
      interactionBoundingBox.max.x = interactionBoundingBox.min.x + labelSize.x + ImGui.getStyle().getItemSpacingX() * 2.0f;
      interactionBoundingBox.max.y = interactionBoundingBox.min.y + frameHeight;

      float mousePosX = ImGui.getMousePosX();
      float mousePosY = ImGui.getMousePosY();

      ImGui.setCursorPosY(ImGui.getCursorPosY() + padding.y);
      arrowRenderer.renderArrow(node.getTreeWidgetExpanded() ? ImGuiDir.Down : ImGuiDir.Right, 0.7f, ImGui.getColorU32(ImGuiCol.Text));
      ImGui.setCursorPosY(ImGui.getCursorPosY() - padding.y);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getFontSize() + padding.x);
      ImGui.pushFont(ImGuiTools.getSmallBoldFont());
      ImGui.text(labelText);
      ImGui.popFont();

      if (node.getTreeWidgetExpanded())
      {
         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            render(child);
         }
      }
   }
}
