package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.robotics.EuclidCoreMissingTools;

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

   /**
    * @return clicked
    */
   public boolean render(boolean expanded)
   {
      return render(expanded, false);
   }

   public boolean render(boolean expanded, boolean expandCollapseAll)
   {
      float fontSize = ImGui.getFontSize();

      float scale = 0.7f; // Make parameter if desired
      scale *= fontSize;

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

      // Fix aliasing
      EuclidCoreMissingTools.roundToGivenPrecision(boxTopLeft, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(boxTopRight, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(boxBottomLeft, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(boxBottomRight, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(minusLeft, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(minusRight, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(plusTop, 1.0);
      EuclidCoreMissingTools.roundToGivenPrecision(plusBottom, 1.0);

      float itemWidth = boxTopRight.getX32() - boxTopLeft.getX32();
      isHovered = ImGuiTools.isItemHovered(itemWidth);

      backgroundColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Button);

      cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();

      if (expandCollapseAll)
      {
         ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                                 cursorXDesktopFrame + boxBottomRight.getX32() + 1.0f, cursorYDesktopFrame + boxBottomRight.getY32() + 1.0f,
                                                 backgroundColor);
         lineColor = ImGui.getColorU32(ImGuiCol.Border);
         ImGui.getWindowDrawList().addRect(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                           cursorXDesktopFrame + boxBottomRight.getX32() + 1.0f, cursorYDesktopFrame + boxBottomRight.getY32() + 1.0f,
                                           lineColor);

         boxTopLeft.add(2.0, 2.0);
         boxTopRight.add(2.0, 2.0);
         boxBottomLeft.add(2.0, 2.0);
         boxBottomRight.add(2.0, 2.0);
         minusLeft.add(2.0, 2.0);
         minusRight.add(2.0, 2.0);
         plusTop.add(2.0, 2.0);
         plusBottom.add(2.0, 2.0);
      }

      ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                              cursorXDesktopFrame + boxBottomRight.getX32() + 1.0f, cursorYDesktopFrame + boxBottomRight.getY32() + 1.0f,
                                              backgroundColor);
      lineColor = ImGui.getColorU32(ImGuiCol.Border);
      ImGui.getWindowDrawList().addRect(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                        cursorXDesktopFrame + boxBottomRight.getX32() + 1.0f, cursorYDesktopFrame + boxBottomRight.getY32() + 1.0f,
                                        lineColor);
      // Doing it this way doesn't work as well because lines have gray endpoints for some reason
      //   drawLine(boxTopLeft, boxTopRight);
      //   drawLine(boxTopRight, boxBottomRight);
      //   drawLine(boxBottomRight, boxBottomLeft);
      //   drawLine(boxBottomLeft, boxTopLeft);

      lineColor = ImGui.getColorU32(ImGuiCol.Text);
      drawLine(minusLeft, minusRight);
      if (!expanded)
         drawLine(plusTop, plusBottom);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);

      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   private void drawLine(Point2D32 from, Point2D32 to)
   {
      // addLine for some reason puts gray pixels on the ends -- working around that - @dcalvert
      ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + from.getX32(), cursorYDesktopFrame + from.getY32(),
                                              cursorXDesktopFrame + to.getX32() + 1.0f, cursorYDesktopFrame + to.getY32() + 1.0f,
                                              lineColor);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
