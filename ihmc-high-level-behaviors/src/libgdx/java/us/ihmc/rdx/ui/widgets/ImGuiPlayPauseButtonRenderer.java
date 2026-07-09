package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.rdx.imgui.ImGuiTools;

public class ImGuiPlayPauseButtonRenderer
{
   private final Point2D32 playTop = new Point2D32();
   private final Point2D32 playTip = new Point2D32();
   private final Point2D32 playBottom = new Point2D32();
   private boolean isHovered = false;

   public boolean render(boolean isPlaying)
   {
      return render(isPlaying, ImGui.getFrameHeight());
   }

   public boolean render(boolean isPlaying, float lineHeight)
   {
      float circleDiameter = 0.96f * lineHeight;
      float circleRadius = circleDiameter / 2.0f;
      float itemWidth = lineHeight;

      isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      float centerX = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX() + itemWidth / 2.0f;
      float centerY = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY() + lineHeight / 2.0f;

      int backgroundColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Button);
      int iconColor = ImGui.getColorU32(ImGuiCol.Text);

      ImGui.getWindowDrawList().addCircleFilled(centerX, centerY, circleRadius, backgroundColor);

      float d = circleDiameter;
      if (!isPlaying)
      {
         float shiftX = 0.04f * d;
         playTop.set(-0.19f * d + shiftX, -0.22f * d);
         playTip.set(0.19f * d + shiftX, 0.0f);
         playBottom.set(-0.19f * d + shiftX, 0.22f * d);

         ImGui.getWindowDrawList().addTriangleFilled(centerX + playTop.getX32(), centerY + playTop.getY32(),
                                                     centerX + playTip.getX32(), centerY + playTip.getY32(),
                                                     centerX + playBottom.getX32(), centerY + playBottom.getY32(),
                                                     iconColor);
      }
      else
      {
         float barWidth = 0.13f * d;
         float barHeight = 0.40f * d;
         float gap = 0.10f * d;
         float halfHeight = barHeight / 2.0f;
         float leftBarLeft = centerX - gap / 2.0f - barWidth;
         float leftBarRight = centerX - gap / 2.0f;
         float rightBarLeft = centerX + gap / 2.0f;
         float rightBarRight = centerX + gap / 2.0f + barWidth;

         ImGui.getWindowDrawList().addRectFilled(leftBarLeft, centerY - halfHeight, leftBarRight, centerY + halfHeight, iconColor);
         ImGui.getWindowDrawList().addRectFilled(rightBarLeft, centerY - halfHeight, rightBarRight, centerY + halfHeight, iconColor);
      }

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);
      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
