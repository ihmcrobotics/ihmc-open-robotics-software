package us.ihmc.rdx.ui.graphics.live;

import imgui.internal.ImGui;
import us.ihmc.commons.FormattingTools;
import us.ihmc.tools.thread.Throttler;

public class RDXMessageSizeReadout
{
   private String messageSizeString;
   private final Throttler messageSizeStatusThrottler = new Throttler();

   public void update(int numberOfBytes)
   {
      if (messageSizeStatusThrottler.run(1.0))
      { // Only doing this at 1 Hz to improve readability and because String building and formatting is expensive to do every tick
         String kilobytes = FormattingTools.getFormattedDecimal1D((double) numberOfBytes / 1000.0);
         messageSizeString = String.format("Message size: ~%s KB", kilobytes);
      }
   }

   public void renderImGuiWidgets()
   {
      if (messageSizeString != null)
      {
         ImGui.sameLine();
         ImGui.text(messageSizeString);
      }
   }
}
