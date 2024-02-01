package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;
import us.ihmc.rdx.ui.RDX3DPanel;

public class ImGuiTextOverlay
{
   private final transient ImVec2 textSize = new ImVec2();
   private static final int MARGIN = 10;

   public void render(String text, float drawMinX, float drawMinY, float textPositionX, float textPositionY)
   {
      ImGui.calcTextSize(textSize, text);
      ImGui.setCursorPos(textPositionX + MARGIN, textPositionY + textSize.y + MARGIN);
      float drawStartX = drawMinX + textPositionX;
      float drawStartY = drawMinY + textPositionY;
      ImGui.getWindowDrawList().addRectFilled(drawStartX,
                                              drawStartY,
                                              drawStartX + textSize.x + MARGIN + MARGIN,
                                              drawStartY + textSize.y + MARGIN + MARGIN,
                                              RDX3DPanel.OVERLAY_BACKGROUND_COLOR);
      ImGui.text(text);
   }
}
