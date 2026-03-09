package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;

/**
 * An icon for the goto node.
 */
public class ImGuiSceneActionWidget
{
   private final ImVec2[] polygon = new ImVec2[]
   {
      new ImVec2(-0.410f, 0.500f),
      new ImVec2(0.480f, 0.494f),
      new ImVec2(0.466f, -0.036f),
      new ImVec2(0.373f, -0.032f),
      new ImVec2(0.205f, 0.155f),
      new ImVec2(0.466f, 0.159f),
      new ImVec2(-0.077f, 0.160f),
      new ImVec2(0.088f, -0.029f),
      new ImVec2(0.372f, -0.033f),
      new ImVec2(-0.201f, -0.030f),
      new ImVec2(-0.333f, 0.161f),
      new ImVec2(-0.072f, 0.161f),
      new ImVec2(-0.416f, 0.159f),
      new ImVec2(-0.417f, -0.028f),
      new ImVec2(-0.204f, -0.029f),
      new ImVec2(-0.419f, -0.031f),
      new ImVec2(-0.480f, -0.203f),
      new ImVec2(-0.286f, -0.278f),
      new ImVec2(-0.118f, -0.137f),
      new ImVec2(-0.420f, -0.036f),
      new ImVec2(0.125f, -0.215f),
      new ImVec2(-0.097f, -0.343f),
      new ImVec2(-0.286f, -0.277f),
      new ImVec2(0.107f, -0.413f),
      new ImVec2(0.334f, -0.287f),
      new ImVec2(0.128f, -0.215f),
      new ImVec2(0.420f, -0.317f),
      new ImVec2(0.352f, -0.500f),
      new ImVec2(-0.480f, -0.204f),
      new ImVec2(-0.418f, -0.030f),
      new ImVec2(-0.410f, 0.500f),
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
