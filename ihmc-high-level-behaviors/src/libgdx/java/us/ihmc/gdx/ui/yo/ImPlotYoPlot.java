package us.ihmc.gdx.ui.yo;

import us.ihmc.yoVariables.variable.YoDouble;

public class ImPlotYoPlot
{
   private final ImPlotPlot imPlotPlot;
   private final YoDouble[] variablesToPlot;

   public ImPlotYoPlot(String label, YoDouble... variablesToPlot)
   {
      imPlotPlot = new ImPlotPlot(label);
      this.variablesToPlot = variablesToPlot;
      for (YoDouble yoDouble : variablesToPlot)
      {
         ImPlotDoublePlotLine plotLine = new ImPlotDoublePlotLine(yoDouble.getName());
         imPlotPlot.getPlotLines().add(plotLine);
      }
   }

   public void render(boolean update)
   {
      if (update)
      {
         for (int i = 0; i < variablesToPlot.length; i++)
         {
            YoDouble yoDouble = variablesToPlot[i];
            ImPlotDoublePlotLine doublePlotLine = (ImPlotDoublePlotLine) imPlotPlot.getPlotLines().get(i);
            doublePlotLine.addValue(yoDouble.getValue());
         }
      }
      imPlotPlot.render();
   }
}
