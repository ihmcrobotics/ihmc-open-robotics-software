package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiTools;

/**
 * A clickable behavior root node icon with hover state.
 */
public class ImGuiRootIconWidget
{
   public boolean render()
   {
      float lineHeight = ImGui.getFrameHeight();
      float fontSize = ImGui.getFontSize();
      float scale = 0.7f * fontSize;

      float wide = 0.4f;
      float down = 0.7f;
      float circleSize = 0.21f;
      float itemWidth = 2.0f * scale;

      float offsetX = ImGui.getCursorScreenPosX() + 0.8f * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + 0.3f * fontSize;
      if (lineHeight == ImGui.getFrameHeight())
         offsetY += ImGui.getStyle().getFramePaddingY();

      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);
      if (isHovered)
      {
         int hoverColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);
         ImGui.getWindowDrawList().addCircleFilled(offsetX, offsetY, circleSize * scale, hoverColor);
         ImGui.getWindowDrawList().addCircleFilled((-wide * scale) + offsetX, (down * scale) + offsetY, circleSize * scale, hoverColor);
         ImGui.getWindowDrawList().addCircleFilled((wide * scale) + offsetX, (down * scale) + offsetY, circleSize * scale, hoverColor);
      }

      float xShift = 0.03f;
      float x1 = 0.13f;
      float x2 = 0.34f;
      float y1 = 0.15f;
      float y2 = 0.5f;
      int lineColor = ImGui.getColorU32(ImGuiCol.Text);
      ImGui.getWindowDrawList()
           .addLine(((x1 - xShift) * scale) + offsetX, (y1 * scale) + offsetY, ((x2 - xShift) * scale) + offsetX, (y2 * scale) + offsetY, lineColor);
      ImGui.getWindowDrawList()
           .addLine(((-x1 - xShift) * scale) + offsetX, (y1 * scale) + offsetY, ((-x2 - xShift) * scale) + offsetX, (y2 * scale) + offsetY, lineColor);
      ImGui.getWindowDrawList().addCircle(offsetX, offsetY, circleSize * scale, lineColor);
      ImGui.getWindowDrawList().addCircle((-wide * scale) + offsetX, (down * scale) + offsetY, circleSize * scale, lineColor);
      ImGui.getWindowDrawList().addCircle((wide * scale) + offsetX, (down * scale) + offsetY, circleSize * scale, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }
}
