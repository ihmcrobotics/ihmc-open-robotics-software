package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.ImVec2;

/**
 * This'll keep track of the longest string and line everything to the right
 * of a bunch of text labels.
 */
public class ImGuiLabelledWidgetAligner extends ImGuiVerticalAligner
{
   private final ImVec2 calcTextSize = new ImVec2();

   public void text(String text)
   {
      ImGui.calcTextSize(calcTextSize, text);
      submitCursorX(calcTextSize.x);
      ImGui.text(text);
      ImGui.sameLine();
      ImGui.setCursorPosX(ImGui.getCursorPosX() - calcTextSize.x + getCursorMaxX());
   }
}
