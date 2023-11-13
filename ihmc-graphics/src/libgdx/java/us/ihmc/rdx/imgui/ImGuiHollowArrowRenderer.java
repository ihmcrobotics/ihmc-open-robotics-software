package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
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

   private boolean isHovered = false;

   public void render()
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

      cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      drawLine(arrowheadTop, arrowheadTip);
      drawLine(arrowheadTip, arrowheadBottom);
      drawLine(arrowheadTop, baseTopRight);
      drawLine(arrowheadBottom, baseBottomRight);
      drawLine(baseTopRight, baseTopLeft);
      drawLine(baseTopLeft, baseBottomLeft);
      drawLine(baseBottomLeft, baseBottomRight);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + (arrowheadTip.getX32() - baseTopLeft.getX32()) + ImGui.getStyle().getFramePaddingX() * 2.0f);
   }

   private void drawLine(Point2D32 from, Point2D32 to)
   {
      ImGui.getWindowDrawList().addLine(cursorXDesktopFrame + from.getX32(), cursorYDesktopFrame + from.getY32(),
                                        cursorXDesktopFrame + to.getX32(), cursorYDesktopFrame + to.getY32(), lineColor);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
