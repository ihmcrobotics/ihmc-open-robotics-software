package us.ihmc.rdx.imgui;

import imgui.ImGui;

public class ImGuiVerticalAligner
{
   private float cursorMaxX = 0.0f;

   public void align()
   {
      submitCursorX(ImGui.getCursorPosX());
      setCursorXToAligned();
   }

   protected void submitCursorX(float cursorX)
   {
      cursorMaxX = Math.max(cursorMaxX, cursorX);
   }

   protected void setCursorXToAligned()
   {
      ImGui.setCursorPosX(cursorMaxX);
   }

   protected float getCursorMaxX()
   {
      return cursorMaxX;
   }
}
