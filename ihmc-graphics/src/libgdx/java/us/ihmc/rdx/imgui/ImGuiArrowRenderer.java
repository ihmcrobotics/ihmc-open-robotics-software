package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiDir;
import us.ihmc.euclid.tuple2D.Point2D32;

public class ImGuiArrowRenderer
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 a = new Point2D32();
   private final Point2D32 b = new Point2D32();
   private final Point2D32 c = new Point2D32();
   private final Point2D32 centerA = new Point2D32();
   private final Point2D32 centerB = new Point2D32();
   private final Point2D32 centerC = new Point2D32();

   /** Taken from imgui_draw.cpp */
   public void renderArrow(int imGuiDirection, float scale, int color)
   {
      float fontSize = ImGui.getFontSize();
      float radius = fontSize * 0.40f * scale;

      center.set(0.5f, 0.5f);
      center.scale(fontSize);
      center.setY(center.getY32() * scale);

      float vertex0 = 0.000f;
      float vertex1 = 0.750f;
      float vertex2 = 0.866f;

      if (imGuiDirection == ImGuiDir.Down)
      {
         a.set(+vertex0, +vertex1);
         b.set(-vertex2, -vertex1);
         c.set(+vertex2, -vertex1);
      }
      if (imGuiDirection == ImGuiDir.Right)
      {
         a.set(+vertex1, +vertex0);
         b.set(-vertex1, +vertex2);
         c.set(-vertex1, -vertex2);
      }

      a.scale(radius);
      b.scale(radius);
      c.scale(radius);

      centerA.add(center, a);
      centerB.add(center, b);
      centerC.add(center, c);

      float cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      float cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      ImGui.getWindowDrawList().addTriangleFilled(cursorXDesktopFrame + centerA.getX32(), cursorYDesktopFrame + centerA.getY32(),
                                                  cursorXDesktopFrame + centerB.getX32(), cursorYDesktopFrame + centerB.getY32(),
                                                  cursorXDesktopFrame + centerC.getX32(), cursorYDesktopFrame + centerC.getY32(),
                                                  color);
   }
}
