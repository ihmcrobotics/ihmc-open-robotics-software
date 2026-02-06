package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A clickable robot arm icon with hover state.
 */
public class ImGuiArmIconWidget
{
   private boolean isHovered;
   private RobotSide side;

   public boolean render(RobotSide side, boolean isSelected)
   {
      this.side = side;

      float fontSize = ImGui.getFontSize();
      float scale = fontSize;

      float offsetX = ImGui.getCursorScreenPosX() + 0.0f * scale;
      float offsetY = ImGui.getCursorScreenPosY() + 0.0f * scale + ImGui.getStyle().getFramePaddingY();

      if (side == RobotSide.LEFT)
         offsetX += scale;
      else
         offsetX += 0.9f * scale;

      float lineHeight = ImGui.getFrameHeight();
      float itemWidth = 1.8f * scale;
      isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      if (isHovered || isSelected)
      {
         int hoverColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);
         ImGui.getWindowDrawList().addCircleFilled(offsetX + flip(0.1f ) * scale, offsetY + 0.8f * scale, 0.21f * scale, hoverColor);
         ImGui.getWindowDrawList().addCircleFilled(offsetX + flip(0.75f) * scale, offsetY + 0.5f * scale, 0.23f * scale, hoverColor);
         ImGui.getWindowDrawList().addCircleFilled(offsetX + flip(0.3f ) * scale, offsetY + 0.1f * scale, 0.16f * scale, hoverColor);
      }

      int lineColor = ImGui.getColorU32(ImGuiCol.Text);
      ImGui.getWindowDrawList().addCircle(offsetX + flip(0.1f ) * scale, offsetY + 0.8f * scale, 0.21f * scale, lineColor);
      ImGui.getWindowDrawList().addCircle(offsetX + flip(0.75f) * scale, offsetY + 0.5f * scale, 0.23f * scale, lineColor);
      ImGui.getWindowDrawList().addCircle(offsetX + flip(0.3f ) * scale, offsetY + 0.1f * scale, 0.16f * scale, lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + flip(0.2f ) * scale, offsetY + .91f * scale,
                                        offsetX + flip(0.68f) * scale, offsetY + 0.66f * scale, lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + flip(0.15f) * scale, offsetY + .6f * scale,
                                        offsetX + flip(0.55f) * scale, offsetY + 0.45f * scale, lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + flip(0.55f) * scale, offsetY + .45f * scale,
                                        offsetX + flip(0.27f) * scale, offsetY + 0.2f * scale, lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + flip(0.7f ) * scale, offsetY + .3f * scale,
                                        offsetX + flip(0.4f ) * scale, offsetY + 0.1f * scale, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   private float flip(float in)
   {
      return side == RobotSide.RIGHT ? in : -in;
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
