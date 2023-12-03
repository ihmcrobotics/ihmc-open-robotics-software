package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiDir;
import us.ihmc.euclid.tuple2D.Point2D32;

public class ImGuiDirectionalTriangleRenderer
{
   private final Point2D32 center = new Point2D32();
   private final Point2D32 a = new Point2D32();
   private final Point2D32 b = new Point2D32();
   private final Point2D32 c = new Point2D32();

   /** Taken from imgui_draw.cpp */
   public void render(int imGuiDirection, float scale, int color)
   {
      float framePaddingY = ImGui.getStyle().getFramePaddingY();
      ImGui.setCursorPosY(ImGui.getCursorPosY() + framePaddingY);

      float fontSize = ImGui.getFontSize();
      float radius = fontSize * 0.4f * scale;

      center.set(0.5f, 0.5f);
      center.scale(fontSize);
      center.setY(center.getY32() * scale);

      float zero = 0.0f;
      float halfheight = 0.75f;
      float halfwidth = 0.866f;

      if (imGuiDirection == ImGuiDir.Down)
      {
         a.set(+zero, +halfheight);
         b.set(-halfwidth, -halfheight);
         c.set(+halfwidth, -halfheight);
      }
      if (imGuiDirection == ImGuiDir.Right)
      {
         a.set(+halfheight, +zero);
         b.set(-halfheight, +halfwidth);
         c.set(-halfheight, -halfwidth);
      }

      a.scaleAdd(radius, center);
      b.scaleAdd(radius, center);
      c.scaleAdd(radius, center);

      float cursorXDesktopFrame = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      float cursorYDesktopFrame = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      ImGui.getWindowDrawList().addTriangleFilled(cursorXDesktopFrame + a.getX32(), cursorYDesktopFrame + a.getY32(),
                                                  cursorXDesktopFrame + b.getX32(), cursorYDesktopFrame + b.getY32(),
                                                  cursorXDesktopFrame + c.getX32(), cursorYDesktopFrame + c.getY32(),
                                                  color);

      ImGui.setCursorPosY(ImGui.getCursorPosY() - framePaddingY);
   }
}
