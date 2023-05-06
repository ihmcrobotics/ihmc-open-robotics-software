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

   private final FrequencyCalculator fpsCalculator = new FrequencyCalculator();
   private final Throttler throttler = new Throttler().setFrequency(1.0);
   private final ImGuiMovingPlot plot = new ImGuiMovingPlot("###frameRateDisplay", 100, 100, (int) ImGuiTools.TAB_BAR_HEIGHT);
   private String fpsString = "0 Hz";

   public void render()
   {
      fpsCalculator.ping();

      plot.setNextValue((float) fpsCalculator.getFrequency());
      plot.calculate("", true);

      if (throttler.run())
      {
         fpsString = String.valueOf((int) fpsCalculator.getFrequency());
         while (fpsString.length() < 3)
         {
            fpsString = " " + fpsString;
         }
         fpsString += " Hz";
      }
      ImGui.text(fpsString);
   }
}
