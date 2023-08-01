package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.rdx.ui.RDX3DPanel;

public class ImGuiTextOverlay
{
   private final transient ImVec2 textSize = new ImVec2();

   public void render(String text, float drawMinX, float drawMinY, float textPositionX, float textPositionY)
   {
      ImGui.calcTextSize(textSize, text);
      int margin = 10;
      ImGui.setCursorPos(textPositionX + margin, textPositionY + textSize.y + margin);
      float drawStartX = drawMinX + textPositionX;
      float drawStartY = drawMinY + textPositionY;
      ImGui.getWindowDrawList().addRectFilled(drawStartX,
                                              drawStartY,
                                              drawStartX + textSize.x + margin + margin,
                                              drawStartY + textSize.y + margin + margin,
                                              RDX3DPanel.OVERLAY_BACKGROUND_COLOR);
      ImGui.text(text);
   }
}
