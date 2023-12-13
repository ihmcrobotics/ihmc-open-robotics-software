package us.ihmc.rdx.ui;

import us.ihmc.rdx.imgui.ImGuiAveragedFrequencyText;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.tools.time.FrequencyCalculator;

/**
 * This is the code for the display in the menu bar of all
 * RDX apps. Not meant to be usable elsewhere.
 */
public class RDXBaseUIFrameRateDisplay
{
   private final ImGuiAveragedFrequencyText averagedFrequencyText = new ImGuiAveragedFrequencyText();
   private final FrequencyCalculator latestFPSCalculator = new FrequencyCalculator(1);
   private final ImGuiMovingPlot plot = new ImGuiMovingPlot("###frameRateDisplay", ImGuiAveragedFrequencyText.HISTORY, 100, (int) ImGuiTools.TAB_BAR_HEIGHT);

   public void renderPlot()
   {
      plot.setNextValue((float) latestFPSCalculator.getFrequency());
      plot.calculate("", true);
   }

   public void renderHz()
   {
      latestFPSCalculator.ping();
      averagedFrequencyText.ping();
      averagedFrequencyText.render();
   }
}
