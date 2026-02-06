package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;

/**
 * An icon for the goto node.
 */
public class ImGuiGotoNodeWidget
{
   private final ImVec2[] polygon = new ImVec2[]
   {
      new ImVec2(0.461f, 0.495f),
      new ImVec2(0.461f, 0.28f),
      new ImVec2(0.108f, 0.266f),
      new ImVec2(-0.022f, 0.179f),
      new ImVec2(-0.030f, -0.010f),
      new ImVec2(0.140f, -0.029f),
      new ImVec2(0.140f, -0.099f),
      new ImVec2(-0.159f, -0.500f),
      new ImVec2(-0.461f, -0.095f),
      new ImVec2(-0.459f, -0.015f),
      new ImVec2(-0.292f, -0.010f),
      new ImVec2(-0.301f, 0.139f),
      new ImVec2(-0.263f, 0.351f),
      new ImVec2(-0.093f, 0.482f),
      new ImVec2(0.177f, 0.500f),
      new ImVec2(0.461f, 0.495f),
   };

   public void render()
   {
      float fontSize = ImGui.getFontSize();
      float scale = fontSize;
      float offsetX = ImGui.getCursorScreenPosX() + 0.5f * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + 0.4f * fontSize + ImGui.getStyle().getFramePaddingY();
      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

      for (int i = 0; i < polygon.length - 1; i++)
         ImGui.getWindowDrawList().addLine(polygon[i].x * scale + offsetX, polygon[i].y * scale + offsetY,
                                           polygon[i + 1].x * scale + offsetX, polygon[i + 1].y * scale + offsetY, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + scale);
      ImGui.newLine();
   }
}
