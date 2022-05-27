package us.ihmc.gdx.ui.tools;

import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;
import us.ihmc.tools.time.FrequencyCalculator;

public class ImPlotFrequencyPlot
{
   private final ImPlotPlot plot = new ImPlotPlot(70);
   private final ImPlotDoublePlotLine plotLine;
   private final FrequencyCalculator frequencyCalculator = new FrequencyCalculator();

   public ImPlotFrequencyPlot(String name)
   {
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
