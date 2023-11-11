package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiDir;

public class ImGuiArrowRenderer
{
   private final ImVec2 center = new ImVec2();
   private final ImVec2 a = new ImVec2();
   private final ImVec2 b = new ImVec2();
   private final ImVec2 c = new ImVec2();
   private final ImVec2 centerA = new ImVec2();
   private final ImVec2 centerB = new ImVec2();
   private final ImVec2 centerC = new ImVec2();

   /** Taken from imgui_draw.cpp */
   public void renderArrow(int imGuiDirection, float scale, int color)
   {
      float h = ImGui.getFontSize();
      float r = h * 0.40f * scale;
      center.set(h * 0.50f, h * 0.50f * scale);

      if (imGuiDirection == ImGuiDir.Down)
      {
         a.set(+0.000f * r, +0.750f * r);
         b.set(-0.866f * r, -0.750f * r);
         c.set(+0.866f * r, -0.750f * r);
      }
      if (imGuiDirection == ImGuiDir.Right)
      {
         a.set(+0.750f * r, +0.000f * r);
         b.set(-0.750f * r, +0.866f * r);
         c.set(-0.750f * r, -0.866f * r);
      }

      centerA.set(center.x + a.x, center.y + a.y);
      centerB.set(center.x + b.x, center.y + b.y);
      centerC.set(center.x + c.x, center.y + c.y);

      float actualCursorX = ImGui.getWindowPosX() + ImGui.getCursorPosX() - ImGui.getScrollX();
      float actualCursorY = ImGui.getWindowPosY() + ImGui.getCursorPosY() - ImGui.getScrollY();
      ImGui.getWindowDrawList().addTriangleFilled(actualCursorX + centerA.x, actualCursorY + centerA.y,
                                                  actualCursorX + centerB.x, actualCursorY + centerB.y,
                                                  actualCursorX + centerC.x, actualCursorY + centerC.y,
                                                  color);
   }
}
