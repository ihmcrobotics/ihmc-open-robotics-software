package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;

public class ImGuiHollowArrowRenderer
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 arrowheadTip = new Point2D32();
   private final Point2D32 arrowheadTop = new Point2D32();
   private final Point2D32 arrowheadBottom = new Point2D32();
   private final Point2D32 baseTopLeft = new Point2D32();
   private final Point2D32 baseTopRight = new Point2D32();
   private final Point2D32 baseBottomLeft = new Point2D32();
   private final Point2D32 baseBottomRight = new Point2D32();
   private float cursorXDesktopFrame;
   private float cursorYDesktopFrame;
   private int lineColor;
   private int backgroundColor;

   private boolean isHovered = false;

   public boolean render(boolean active)
   {
      lineColor = ImGui.getColorU32(ImGuiCol.Text);

      float fontSize = ImGui.getFontSize();

      float scale = 0.7f; // Make parameter if desired
      scale *= fontSize;

      // Center is base of arrowhead
      center.set(0.7f * fontSize, 0.5f * fontSize);

      float zero = 0.0f;
      float arrowheadHalfheight = 0.4f;
      float arrowheadHalfwidth = 0.6f;
      float baseWidth = 0.9f;
      float baseHeight = 0.35f;

      // a, b, c are arrowhead points
      arrowheadTop.set(+zero, -arrowheadHalfheight);
      arrowheadTip.set(+arrowheadHalfwidth, +zero);
      arrowheadBottom.set(+zero, +arrowheadHalfheight);

      baseTopLeft.set(-baseWidth, -baseHeight / 2.0f);
      baseTopRight.set(+zero, -baseHeight / 2.0f);
      baseBottomLeft.set(-baseWidth, +baseHeight / 2.0f);
      baseBottomRight.set(+zero, +baseHeight / 2.0f);

      arrowheadTop.scaleAdd(scale, center);
      arrowheadTip.scaleAdd(scale, center);
      arrowheadBottom.scaleAdd(scale, center);
      baseTopLeft.scaleAdd(scale, center);
      baseTopRight.scaleAdd(scale, center);
      baseBottomLeft.scaleAdd(scale, center);
      baseBottomRight.scaleAdd(scale, center);

      float itemWidth = arrowheadTip.getX32() - baseTopLeft.getX32();
      isHovered = ImGuiTools.isItemHovered(itemWidth);

      cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();

      if (active || isHovered)
      {
         backgroundColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGuiTools.GREEN;

         ImGui.getWindowDrawList() .addTriangleFilled(cursorXDesktopFrame + arrowheadTop.getX32(), cursorYDesktopFrame + arrowheadTop.getY32(),
                                                      cursorXDesktopFrame + arrowheadTip.getX32(), cursorYDesktopFrame + arrowheadTip.getY32(),
                                                      cursorXDesktopFrame + arrowheadBottom.getX32(), cursorYDesktopFrame + arrowheadBottom.getY32(),
                                                      backgroundColor);
         ImGui.getWindowDrawList() .addRectFilled(cursorXDesktopFrame + baseTopLeft.getX32(), cursorYDesktopFrame + baseTopLeft.getY32(),
                                                  cursorXDesktopFrame + baseBottomRight.getX32() + 1.0f, cursorYDesktopFrame + baseBottomRight.getY32() + 1.0f,
                                                  backgroundColor);
      }

      drawLine(arrowheadTop, arrowheadTip);
      drawLine(arrowheadTip, arrowheadBottom);
      drawLine(arrowheadTop, baseTopRight);
      drawLine(arrowheadBottom, baseBottomRight);
      drawLine(baseTopRight, baseTopLeft);
      drawLine(baseTopLeft, baseBottomLeft);
      drawLine(baseBottomLeft, baseBottomRight);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);

      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   private void drawLine(Point2D32 from, Point2D32 to)
   {
      ImGui.getWindowDrawList().addLine(cursorXDesktopFrame + from.getX32(), cursorYDesktopFrame + from.getY32(),
                                        cursorXDesktopFrame + to.getX32(), cursorYDesktopFrame + to.getY32(),
                                        lineColor);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
