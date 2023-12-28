package us.ihmc.rdx.imgui;

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

   public void renderImGuiWidgets(float plotWidth, float plotHeight)
   {
      plot.render(plotWidth, plotHeight);
   }

   public ImPlotDoublePlotLine getPlotLine()
   {
      return plotLine;
   }
}
