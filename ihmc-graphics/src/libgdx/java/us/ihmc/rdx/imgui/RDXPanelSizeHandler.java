package us.ihmc.rdx.imgui;

import imgui.flag.ImGuiCond;
import imgui.internal.ImGui;

/**
 * This class makes the panels smaller when you undock them because otherwise it's
 * difficult to move them around and re-dock them. It also makes the panel's initial size
 * to be smallish.
 */
public class RDXPanelSizeHandler
{
   private int firstTimeWidth = 300;
   private int firstTimeHeight = 200;
   private boolean wasDocked = false;
   private boolean setNextWindowSize = false;

   public void handleSizeBeforeBegin()
   {
      ImGui.setNextWindowSize(firstTimeWidth, firstTimeHeight, ImGuiCond.FirstUseEver);
      if (setNextWindowSize)
      {
         setNextWindowSize = false;
         ImGui.setNextWindowSize(firstTimeWidth, firstTimeHeight);
      }
   }

   public void handleSizeAfterBegin()
   {
      boolean isDocked = ImGui.isWindowDocked();
      if (wasDocked && !isDocked)
      {
         setNextWindowSize = true;
      }
      wasDocked = isDocked;
   }

   public void setFirstTimeWidth(int firstTimeWidth)
   {
      this.firstTimeWidth = firstTimeWidth;
   }

   public void setFirstTimeHeight(int firstTimeHeight)
   {
      this.firstTimeHeight = firstTimeHeight;
   }
}
