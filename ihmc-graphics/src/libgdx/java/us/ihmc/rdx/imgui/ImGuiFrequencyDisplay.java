package us.ihmc.rdx.imgui;

import us.ihmc.tools.time.FrequencyCalculator;

/**
 * Call {@link #ping} at some frequency and optionally render a size 100 window
 * text in the form of "10 Hz" and/or a plot that does no averaging.
 */
public class ImGuiFrequencyDisplay
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiAveragedFrequencyText averagedFrequencyText = new ImGuiAveragedFrequencyText();
   private final FrequencyCalculator plotFrequencyCalculator = new FrequencyCalculator();
   private final ImGuiMovingPlot plot;

   public ImGuiFrequencyDisplay(String hiddenLabel)
   {
      plot = new ImGuiMovingPlot(labels.getHidden(hiddenLabel), ImGuiAveragedFrequencyText.HISTORY, 100, (int) ImGuiTools.TAB_BAR_HEIGHT);
   }

   public void ping()
   {
      plotFrequencyCalculator.ping();
      averagedFrequencyText.ping();
   }

   public void renderPlot()
   {
      plot.setNextValue((float) plotFrequencyCalculator.getFrequency());
      plot.calculate("", true);
   }

   public void renderHz()
   {
      averagedFrequencyText.render();
   }
}
