package us.ihmc.rdx.imgui;

import us.ihmc.tools.time.FrequencyCalculator;

public class ImPlotFrequencyPlot
{
   private final ImPlotPlot plot;
   private final ImPlotDoublePlotLine plotLine;
   private final FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

   public ImPlotFrequencyPlot(String name)
   {
      this(name, 70);
   }

   public ImPlotFrequencyPlot(String name, int heightInPixels)
   {
      plot = new ImPlotPlot(heightInPixels);
      plotLine = new ImPlotDoublePlotLine(name);
      plot.getPlotLines().add(plotLine);
   }

   public void ping()
   {
      frequencyCalculator.ping();
   }

   public void renderImGuiWidgets()
   {
      plotLine.addValue(frequencyCalculator.getFrequency());
      plot.render();
   }

   public boolean anyPingsYet()
   {
      return frequencyCalculator.anyPingsYet();
   }
}
