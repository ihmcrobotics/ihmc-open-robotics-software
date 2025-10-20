package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;

/**
 * An icon for the goto node.
 */
public class ImGuiCheckpointNodeWidget
{
   private final ImVec2[] polygon = new ImVec2[]
   {
      new ImVec2(-0.316f, 0.500f),
      new ImVec2(-0.315f, -0.359f),
      new ImVec2(-0.352f, -0.375f),
      new ImVec2(-0.364f, -0.436f),
      new ImVec2(-0.328f, -0.499f),
      new ImVec2(-0.240f, -0.500f),
      new ImVec2(-0.191f, -0.459f),
      new ImVec2(-0.191f, -0.389f),
      new ImVec2(-0.223f, -0.355f),
      new ImVec2(-0.307f, -0.355f),
      new ImVec2(-0.227f, -0.353f),
      new ImVec2(0.364f, -0.073f),
      new ImVec2(-0.223f, 0.096f),
      new ImVec2(-0.228f, -0.352f),
      new ImVec2(-0.227f, 0.500f),
      new ImVec2(-0.316f, 0.500f),
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
