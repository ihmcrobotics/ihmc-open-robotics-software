package us.ihmc.rdx.ui;

import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.tools.time.FrequencyCalculator;

/**
 * This is the code for the display in the menu bar of all
 * RDX apps. Not meant to be usable elsewhere.
 */
public class RDXBaseUIFrameRateDisplay
{

   private static final int HISTORY = 100;
   private final FrequencyCalculator latestFPSCalculator = new FrequencyCalculator(1);
   private final FrequencyCalculator averagedFPSCalculator = new FrequencyCalculator(HISTORY);
   private final Throttler throttler = new Throttler().setFrequency(1.0);
   private final ImGuiMovingPlot plot = new ImGuiMovingPlot("###frameRateDisplay", HISTORY, 100, (int) ImGuiTools.TAB_BAR_HEIGHT);

   private String fpsString = "0 Hz";

   public void renderPlot()
   {
      plot.setNextValue((float) latestFPSCalculator.getFrequency());
      plot.calculate("", true);
   }

   public void renderHz()
   {
      latestFPSCalculator.ping();
      averagedFPSCalculator.ping();

      if (throttler.run())
      {
         fpsString = String.valueOf((int) averagedFPSCalculator.getFrequency());
         while (fpsString.length() < 3)
         {
            fpsString = " " + fpsString;
         }
         fpsString += " Hz";
      }
      ImGui.text(fpsString);
   }
}
