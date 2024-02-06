package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.tuple2D.Point2D32;

public class ImGuiExpandCollapseRenderer
{
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
      return render(expanded, expandCollapseAll, ImGui.getFontSize());
   }

   public boolean render(boolean expanded, boolean expandCollapseAll, float lineHeight)
   {
      float itemSize = ImGui.getFontSize() * 0.8f;

      int boxSize = (int) Math.floor(itemSize);
      if (boxSize % 2 == 0)
         ++boxSize;

      float boxSizePixels = (float) boxSize;
      float pixelsToCenter = (int) Math.floor(boxSize / 2.0);
      int minusWidth = (int) Math.floor(boxSizePixels * 0.6);
      if (minusWidth % 2 == 0)
         ++minusWidth;
      float minusWidthPixels = (float) minusWidth;
      float pixelsToMinus = (boxSizePixels - minusWidthPixels) / 2.0f;

      float zero = 0.0f;

      boxTopLeft.set(zero, zero);
      boxTopRight.set(boxSizePixels, zero);
      boxBottomLeft.set(zero, boxSizePixels);
      boxBottomRight.set(boxSizePixels, boxSizePixels);
      minusLeft.set(pixelsToMinus, pixelsToCenter);
      minusRight.set(pixelsToMinus + minusWidthPixels, pixelsToCenter + 1.0f);
      plusTop.set(pixelsToCenter, pixelsToMinus);
      plusBottom.set(pixelsToCenter + 1.0f, pixelsToMinus + minusWidthPixels);

      float centering = (float) Math.floor((lineHeight - itemSize) / 2.0f);
      shiftAll(centering, centering);

      float itemWidth = boxTopRight.getX32() - boxTopLeft.getX32();
      isHovered = ImGuiTools.isItemHovered(itemWidth, lineHeight);

      backgroundColor = isHovered ? ImGui.getColorU32(ImGuiCol.ButtonHovered) : ImGui.getColorU32(ImGuiCol.Button);

      cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();

      if (expandCollapseAll)
      {
         ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                                 cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
                                                 backgroundColor);
         lineColor = ImGui.getColorU32(ImGuiCol.Border);
         ImGui.getWindowDrawList().addRect(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                           cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
                                           lineColor);
         shiftAll(2.0f, 2.0f);
      }

      ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                              cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
                                              backgroundColor);
      lineColor = ImGui.getColorU32(ImGuiCol.Border);
      ImGui.getWindowDrawList().addRect(cursorXDesktopFrame + boxTopLeft.getX32(), cursorYDesktopFrame + boxTopLeft.getY32(),
                                        cursorXDesktopFrame + boxBottomRight.getX32(), cursorYDesktopFrame + boxBottomRight.getY32(),
                                        lineColor);

      lineColor = ImGui.getColorU32(ImGuiCol.Text);
      drawLine(minusLeft, minusRight);
      if (!expanded)
         drawLine(plusTop, plusBottom);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + itemWidth);

      ImGui.newLine();

      return isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left);
   }

   private void shiftAll(float x, float y)
   {
      boxTopLeft.add(x, y);
      boxTopRight.add(x, y);
      boxBottomLeft.add(x, y);
      boxBottomRight.add(x, y);
      minusLeft.add(x, y);
      minusRight.add(x, y);
      plusTop.add(x, y);
      plusBottom.add(x, y);
   }

   private void drawLine(Point2D32 from, Point2D32 to)
   {
      // addLine for some reason puts gray pixels on the ends -- working around that - @dcalvert
      ImGui.getWindowDrawList().addRectFilled(cursorXDesktopFrame + from.getX32(), cursorYDesktopFrame + from.getY32(),
                                              cursorXDesktopFrame + to.getX32(), cursorYDesktopFrame + to.getY32(),
                                              lineColor);
   }

   public boolean getIsHovered()
   {
      return isHovered;
   }
}
