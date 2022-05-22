package us.ihmc.gdx.ui.tools;

import us.ihmc.gdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;

public class ImPlotDoublePlot
{
   private final ImPlotPlot plot;
   private final ImPlotDoublePlotLine plotLine;

   public ImPlotDoublePlot(String name)
   {
      this(name, 70);
   }

   public ImPlotDoublePlot(String name, int heightInPixels)
   {
      plot = new ImPlotPlot(heightInPixels);
      plotLine = new ImPlotDoublePlotLine(name);
      plot.getPlotLines().add(plotLine);
   }

   public void addValue(double value)
   {
      plotLine.addValue(value);
   }

   public void renderImGuiWidgets()
   {
      plot.render();
   }
}
