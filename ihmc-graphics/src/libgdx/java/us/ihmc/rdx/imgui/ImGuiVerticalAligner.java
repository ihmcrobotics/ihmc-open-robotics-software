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

      // FIXME: I don't know why, but this values runs away when docking/undocking panels
      // TODO: See if it goes away in future ImGui version? - @dcalvert
      if (cursorMaxX > 250)
         cursorMaxX = 0;
   }

   public void setCursorXToAligned()
   {
      ImGui.setCursorPosX(cursorMaxX);
   }

   public float getCursorMaxX()
   {
      return cursorMaxX;
   }
}
