package us.ihmc.gdx.ui.yo;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class ImPlotYoPlot
{
   private final ImPlotPlot imPlotPlot;
   private final YoVariable[] variablesToPlot;

   public ImPlotYoPlot(YoVariable... variablesToPlot)
   {
      imPlotPlot = new ImPlotPlot();
      this.variablesToPlot = variablesToPlot;
      for (YoVariable yoVariable : variablesToPlot)
      {
         if (yoVariable instanceof YoDouble)
         {
            YoDouble yoDouble = (YoDouble) yoVariable;
            ImPlotDoublePlotLine plotLine = new ImPlotDoublePlotLine(yoDouble.getName());
            imPlotPlot.getPlotLines().add(plotLine);
         }
         else if (yoVariable instanceof YoInteger)
         {
            YoInteger yoInteger = (YoInteger) yoVariable;
            ImPlotIntegerPlotLine plotLine = new ImPlotIntegerPlotLine(yoInteger.getName());
            imPlotPlot.getPlotLines().add(plotLine);
         }
      }
   }

   public void render(boolean update)
   {
      updateVariables(update);
      imPlotPlot.render();
   }

   public void render(float plotWidth, float plotHeight, boolean update)
   {
      updateVariables(update);
      imPlotPlot.render(plotWidth, plotHeight);
   }

   private void updateVariables(boolean update)
   {
      if (update)
      {
         for (int i = 0; i < variablesToPlot.length; i++)
         {
            YoVariable yoVariable = variablesToPlot[i];
            if (yoVariable instanceof YoDouble)
            {
               YoDouble yoDouble = (YoDouble) yoVariable;
               ImPlotDoublePlotLine doublePlotLine = (ImPlotDoublePlotLine) imPlotPlot.getPlotLines().get(i);
               doublePlotLine.addValue(yoDouble.getValue());
            }
            else if (yoVariable instanceof YoInteger)
            {
               YoInteger yoInteger = (YoInteger) yoVariable;
               ImPlotIntegerPlotLine integerPlotLine = (ImPlotIntegerPlotLine) imPlotPlot.getPlotLines().get(i);
               integerPlotLine.addValue(yoInteger.getValue());
            }
         }
      }
   }
}
