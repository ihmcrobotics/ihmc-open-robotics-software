package us.ihmc.gdx.ui.tools;

import us.ihmc.gdx.ui.yo.ImPlotIntegerPlotLine;
import us.ihmc.gdx.ui.yo.ImPlotPlot;

public class ImPlotIntegerPlot
{
   private final ImPlotPlot plot;
   private final ImPlotIntegerPlotLine plotLine;

   public ImPlotIntegerPlot(String name)
   {
      this(name, 70);
   }

   public ImPlotIntegerPlot(String name, int heightInPixels)
   {
      plot = new ImPlotPlot(heightInPixels);
      plotLine = new ImPlotIntegerPlotLine(name);
      plot.getPlotLines().add(plotLine);
   }

   public void addValue(int value)
   {
      plotLine.addValue(value);
   }

   public void renderImGuiWidgets()
   {
      plot.render();
   }
}
