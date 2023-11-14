package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.tools.Timer;

/**
 * Flashes text between black and a color of choice.
 */
public class ImGuiFlashingButton
{
   private static final double PERIOD = 1.0;
   private final Timer timer = new Timer();
   private boolean flashState = false;
   private final int flashColor;

   public ImGuiFlashingButton(int flashColor)
   {
      this.flashColor = flashColor;
   }

   public boolean renderButton(String text, boolean flashing)
   {
      if (!timer.isRunning(PERIOD))
      {
         timer.reset();
         flashState = !flashState;
      }

      if (flashing && flashState)
      {
         ImGui.pushStyleColor(ImGuiCol.Button, flashColor);
      }
      boolean value = ImGui.button(text);
      if (flashing && flashState)
      {
         ImGui.popStyleColor();
      }
      return value;
   }
}
