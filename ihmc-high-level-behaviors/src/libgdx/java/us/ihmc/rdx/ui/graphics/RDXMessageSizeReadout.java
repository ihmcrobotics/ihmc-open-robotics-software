package us.ihmc.rdx.ui.graphics;

import imgui.internal.ImGui;
import us.ihmc.commons.FormattingTools;
import us.ihmc.tools.thread.Throttler;

/**
 * Keep track of the size of the incoming messages while slowing it down
 * to reduce String manipulation and allow the user to read values without them
 * switching too quickly. Extracted to simplify visualizers.
 */
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
         ImGui.text(messageSizeString);
      }
   }
}
