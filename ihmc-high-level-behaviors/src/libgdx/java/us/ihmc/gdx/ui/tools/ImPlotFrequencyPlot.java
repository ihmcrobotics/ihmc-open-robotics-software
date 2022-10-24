package us.ihmc.gdx.ui.tools;

import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
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
}
