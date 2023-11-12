package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiDir;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiArrowRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXBehaviorTreeWidgetRenderer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiArrowRenderer arrowRenderer = new ImGuiArrowRenderer();
   private final ImVec2 padding = new ImVec2();
   private final ImVec2 labelSize = new ImVec2();

   public void render(RDXBehaviorTreeNode<?, ?> node)
   {
      String labelText = node.getDefinition().getDescription();

      padding.set(ImGui.getStyle().getFramePaddingX(), ImGui.getStyle().getFramePaddingY());
      ImGui.calcTextSize(labelSize, labelText);

      float frameHeight = labelSize.y + padding.y;

      float mousePosXInDesktopFrame = ImGui.getMousePosX();
      float mousePosYInDesktopFrame = ImGui.getMousePosY();
      // Widget frame is the top-left of the start of the widgets, which is not the same as window
      // frame in the case the window is scrolled.
      float mousePosXInWidgetFrame = mousePosXInDesktopFrame - ImGui.getWindowPosX() + ImGui.getScrollX();
      float mousePosYInWidgetFrame = mousePosYInDesktopFrame - ImGui.getWindowPosY() + ImGui.getScrollY();

      boolean isHoveringArrow = mousePosXInWidgetFrame >= ImGui.getCursorPosX();
      isHoveringArrow &= mousePosXInWidgetFrame <= ImGui.getCursorPosX() + ImGui.getFontSize() + padding.x;
      isHoveringArrow &= mousePosYInWidgetFrame >= ImGui.getCursorPosY();
      isHoveringArrow &= mousePosYInWidgetFrame <= ImGui.getCursorPosY() + frameHeight;

      if (ImGui.isWindowHovered() && isHoveringArrow && ImGui.isMouseClicked(ImGuiMouseButton.Left))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      int arrowColor = isHoveringArrow ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Text);

      ImGui.setCursorPosY(ImGui.getCursorPosY() + padding.y);
      arrowRenderer.renderArrow(node.getTreeWidgetExpanded() ? ImGuiDir.Down : ImGuiDir.Right, 0.7f, arrowColor);
      ImGui.setCursorPosY(ImGui.getCursorPosY() - padding.y);

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
