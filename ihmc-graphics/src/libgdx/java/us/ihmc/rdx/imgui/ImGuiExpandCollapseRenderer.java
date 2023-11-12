package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.tuple2D.Point2D32;

public class ImGuiExpandCollapseRenderer
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 boxTopLeft = new Point2D32();
   private final Point2D32 boxTopRight = new Point2D32();
   private final Point2D32 boxBottomLeft = new Point2D32();
   private final Point2D32 boxBottomRight = new Point2D32();
   private final Point2D32 minusLeft = new Point2D32();
   private final Point2D32 minusRight = new Point2D32();
   private final Point2D32 plusTop = new Point2D32();
   private final Point2D32 plusBottom = new Point2D32();
   private float cursorXDesktopFrame;
   private float cursorYDesktopFrame;
   private int lineColor;
   private int backgroundColor;

   private boolean isHovered = false;

   public void render(boolean expanded, float scale, int color)
   {
      float fontSize = ImGui.getFontSize();

      scale *= fontSize;

      // Center is base of arrowhead
      center.set(0.5f * fontSize, 0.5f * fontSize);

      float zero = 0.0f;
      float boxHalfsize = 0.5f;
      float plusHalfsize = 0.3f;

      boxTopLeft.set(-boxHalfsize, -boxHalfsize);
      boxTopRight.set(boxHalfsize, -boxHalfsize);
      boxBottomLeft.set(-boxHalfsize, +boxHalfsize);
      boxBottomRight.set(+boxHalfsize, +boxHalfsize);
      minusLeft.set(-plusHalfsize, zero);
      minusRight.set(+plusHalfsize, zero);
      plusTop.set(zero, -plusHalfsize);
      plusBottom.set(zero, +plusHalfsize);

      boxTopLeft.scaleAdd(scale, center);
      boxTopRight.scaleAdd(scale, center);
      boxBottomLeft.scaleAdd(scale, center);
      boxBottomRight.scaleAdd(scale, center);
      minusLeft.scaleAdd(scale, center);
      minusRight.scaleAdd(scale, center);
      plusTop.scaleAdd(scale, center);
      plusBottom.scaleAdd(scale, center);

      float mousePosXInDesktopFrame = ImGui.getMousePosX();
      float mousePosYInDesktopFrame = ImGui.getMousePosY();
      // Widget frame is the top-left of the start of the widgets, which is not the same as window
      // frame in the case the window is scrolled.
      float mousePosXInWidgetFrame = mousePosXInDesktopFrame - ImGui.getWindowPosX() + ImGui.getScrollX();
      float mousePosYInWidgetFrame = mousePosYInDesktopFrame - ImGui.getWindowPosY() + ImGui.getScrollY();

      isHovered = mousePosXInWidgetFrame >= ImGui.getCursorPosX();
      isHovered &= mousePosXInWidgetFrame <= ImGui.getCursorPosX() + ImGui.getFontSize() + ImGui.getStyle().getFramePaddingX();
      isHovered &= mousePosYInWidgetFrame >= ImGui.getCursorPosY();
      isHovered &= mousePosYInWidgetFrame <= ImGui.getCursorPosY() + ImGui.getFontSize() + ImGui.getStyle().getFramePaddingY();

      lineColor = ImGui.getColorU32(ImGuiCol.Text);
      backgroundColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Button);

      cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                              cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
                                              backgroundColor);
//      ImGui.getWindowDrawList().addRect(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
//                                        cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
//                                        lineColor);

      drawLine(boxTopLeft, boxTopRight);
      drawLine(boxTopRight, boxBottomRight);
      drawLine(boxBottomRight, boxBottomLeft);
      drawLine(boxBottomLeft, boxTopLeft);
      drawLine(minusLeft, minusRight);
      if (!expanded)
         drawLine(plusTop, plusBottom);
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
