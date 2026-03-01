package us.ihmc.rdx.ui.widgets;

import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;

/**
 * An icon for the door node.
 */
public class ImGuiDoorNodeWidget
{
   private final ImVec2[] openDoorPart = new ImVec2[]
   {
      new ImVec2(0.284f, 0.435f),
      new ImVec2(0.178f, 0.435f),
      new ImVec2(-0.284f, 0.500f),
      new ImVec2(-0.280f, -0.500f),
      new ImVec2(0.181f, -0.447f),
      new ImVec2(0.185f, 0.435f),
   };
   private final ImVec2[] framePart = new ImVec2[]
   {
      new ImVec2(0.167f, 0.500f),
      new ImVec2(-0.174f, 0.500f),
      new ImVec2(-0.062f, 0.500f),
      new ImVec2(-0.069f, -0.498f),
      new ImVec2(0.174f, -0.500f),
      new ImVec2(0.174f, -0.421f),
      new ImVec2(0.018f, -0.423f),
      new ImVec2(0.024f, 0.498f),
      new ImVec2(0.167f, 0.500f),
   };

   public void render()
   {
      float fontSize = ImGui.getFontSize();
      float scale = fontSize;

      float superOffsetX = 0.3f;
      float superOffsetY = 0.05f;

      float offsetX = ImGui.getCursorScreenPosX() + (superOffsetX + 0.65f) * fontSize;
      float offsetY = ImGui.getCursorScreenPosY() + (superOffsetY + 0.4f) * fontSize + ImGui.getStyle().getFramePaddingY();
      int lineColor = ImGui.getColorU32(ImGuiCol.Text);

      for (int i = 0; i < openDoorPart.length - 1; i++)
         ImGui.getWindowDrawList().addLine(openDoorPart[i].x * scale + offsetX, openDoorPart[i].y * scale + offsetY,
                                           openDoorPart[i + 1].x * scale + offsetX, openDoorPart[i + 1].y * scale + offsetY, lineColor);
      scale = 0.9f * fontSize;
      offsetX = ImGui.getCursorScreenPosX() + (superOffsetX + 0.2f) * fontSize;
      offsetY = ImGui.getCursorScreenPosY() + (superOffsetY + 0.4f) * fontSize + ImGui.getStyle().getFramePaddingY();
      for (int i = 0; i < framePart.length - 1; i++)
         ImGui.getWindowDrawList().addLine(framePart[i].x * scale + offsetX, framePart[i].y * scale + offsetY,
                                           framePart[i + 1].x * scale + offsetX, framePart[i + 1].y * scale + offsetY, lineColor);
      ImGui.getWindowDrawList().addCircle((framePart[0].x + 0.2f) * scale + offsetX, (framePart[0].y - 0.4f) * scale + offsetY, 0.07f * fontSize, lineColor);

      ImGui.setCursorPosX(ImGui.getCursorPosX() + scale);
      ImGui.newLine();
   }
}
