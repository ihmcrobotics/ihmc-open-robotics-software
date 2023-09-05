package us.ihmc.rdx.ui.tools;

import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlot;

public class ImPlotStopwatchPlot
{
   private final ImPlotPlot plot = new ImPlotPlot(70);
   private final ImPlotDoublePlotLine plotLine;
   private final Stopwatch stopwatch = new Stopwatch();

   public ImPlotStopwatchPlot(String name)
   {
      plotLine = new ImPlotDoublePlotLine(name);
      plot.getPlotLines().add(plotLine);
   }

   public void start()
   {
      stopwatch.reset();
   }

   public void stop()
   {
      plotLine.addValue(stopwatch.totalElapsed());
   }

   public void renderImGuiWidgets()
   {
      plot.render();
   }
}
