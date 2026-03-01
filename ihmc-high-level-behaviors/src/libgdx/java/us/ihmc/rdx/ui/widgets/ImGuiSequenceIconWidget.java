package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.rdx.imgui.ImGuiTools;

/**
 * A clickable behavior root node icon with hover state.
 */
public class ImGuiSequenceIconWidget
{
   public boolean render()
   {
      return render(ImGui.getFrameHeight());
   }

   public boolean render(float lineHeight)
   {
      float fontSize = ImGui.getFontSize();
      float scale = 0.7f * fontSize;

      float offsetX = ImGui.getCursorScreenPosX() + 1.0f * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + 0.41f * fontSize;
      if (lineHeight == ImGui.getFrameHeight())
         offsetY += ImGui.getStyle().getFramePaddingY();

      float squish = 0.5f;
      float zero = 0.0f;
      float arrowheadHalfheight = 0.4f * squish;
      float arrowheadHalfwidth = 0.6f;
      float baseWidth = 0.9f;
      float baseHeight = 0.35f * squish;
      float itemWidth = 2.3f * scale;
      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

      boolean isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      float arrowheadTopX = zero;
      float arrowheadTopY = -arrowheadHalfheight;
      float arrowheadTipX = arrowheadHalfwidth;
      float arrowheadTipY = zero;
      float arrowheadBottomX = zero;
      float arrowheadBottomY = arrowheadHalfheight;
      float baseTopLeftX = -baseWidth;
      float baseTopLeftY = -baseHeight / 2.0f;
      float baseTopRightX = zero;
      float baseTopRightY = -baseHeight / 2.0f;
      float baseBottomLeftX = -baseWidth;
      float baseBottomLeftY = baseHeight / 2.0f;
      float baseBottomRightX = zero;
      float baseBottomRightY = baseHeight / 2.0f;

      if (isHovered)
      {
         int hoverColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);

         ImGui.getWindowDrawList() .addTriangleFilled(offsetX + scale * arrowheadTopX, offsetY + scale * arrowheadTopY,
                                                      offsetX + scale * arrowheadTipX, offsetY + scale * arrowheadTipY,
                                                      offsetX + scale * arrowheadBottomX, offsetY + scale * arrowheadBottomY,
                                                      hoverColor);
         ImGui.getWindowDrawList() .addRectFilled(offsetX + scale * baseTopLeftX, offsetY + scale * baseTopLeftY,
                                                  offsetX + scale * baseBottomRightX + 1.0f, offsetY + scale * baseBottomRightY + 1.0f,
                                                  hoverColor);
      }

      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTopX,
                                        offsetY + scale * arrowheadTopY,
                                        offsetX + scale * arrowheadTipX,
                                        offsetY + scale * arrowheadTipY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTipX,
                                        offsetY + scale * arrowheadTipY,
                                        offsetX + scale * arrowheadBottomX,
                                        offsetY + scale * arrowheadBottomY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTopX,
                                        offsetY + scale * arrowheadTopY,
                                        offsetX + scale * baseTopRightX,
                                        offsetY + scale * baseTopRightY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadBottomX,
                                        offsetY + scale * arrowheadBottomY,
                                        offsetX + scale * baseBottomRightX,
                                        offsetY + scale * baseBottomRightY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseTopRightX,
                                        offsetY + scale * baseTopRightY,
                                        offsetX + scale * baseTopLeftX,
                                        offsetY + scale * baseTopLeftY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseTopLeftX,
                                        offsetY + scale * baseTopLeftY,
                                        offsetX + scale * baseBottomLeftX,
                                        offsetY + scale * baseBottomLeftY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseBottomLeftX,
                                        offsetY + scale * baseBottomLeftY,
                                        offsetX + scale * baseBottomRightX,
                                        offsetY + scale * baseBottomRightY,
                                        lineColor);
      offsetY += 0.5f * scale;
      if (isHovered)
      {
         int hoverColor = ImGui.getColorU32(ImGuiCol.ButtonHovered);

         ImGui.getWindowDrawList() .addTriangleFilled(offsetX + scale * arrowheadTopX, offsetY + scale * arrowheadTopY,
                                                      offsetX + scale * arrowheadTipX, offsetY + scale * arrowheadTipY,
                                                      offsetX + scale * arrowheadBottomX, offsetY + scale * arrowheadBottomY,
                                                      hoverColor);
         ImGui.getWindowDrawList() .addRectFilled(offsetX + scale * baseTopLeftX, offsetY + scale * baseTopLeftY,
                                                  offsetX + scale * baseBottomRightX + 1.0f, offsetY + scale * baseBottomRightY + 1.0f,
                                                  hoverColor);
      }
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTopX,
                                        offsetY + scale * arrowheadTopY,
                                        offsetX + scale * arrowheadTipX,
                                        offsetY + scale * arrowheadTipY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTipX,
                                        offsetY + scale * arrowheadTipY,
                                        offsetX + scale * arrowheadBottomX,
                                        offsetY + scale * arrowheadBottomY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadTopX,
                                        offsetY + scale * arrowheadTopY,
                                        offsetX + scale * baseTopRightX,
                                        offsetY + scale * baseTopRightY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * arrowheadBottomX,
                                        offsetY + scale * arrowheadBottomY,
                                        offsetX + scale * baseBottomRightX,
                                        offsetY + scale * baseBottomRightY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseTopRightX,
                                        offsetY + scale * baseTopRightY,
                                        offsetX + scale * baseTopLeftX,
                                        offsetY + scale * baseTopLeftY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseTopLeftX,
                                        offsetY + scale * baseTopLeftY,
                                        offsetX + scale * baseBottomLeftX,
                                        offsetY + scale * baseBottomLeftY,
                                        lineColor);
      ImGui.getWindowDrawList().addLine(offsetX + scale * baseBottomLeftX,
                                        offsetY + scale * baseBottomLeftY,
                                        offsetX + scale * baseBottomRightX,
                                        offsetY + scale * baseBottomRightY,
                                        lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();
      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }
}
