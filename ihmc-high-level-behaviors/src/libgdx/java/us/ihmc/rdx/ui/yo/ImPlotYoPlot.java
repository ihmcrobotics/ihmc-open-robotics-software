package us.ihmc.rdx.ui.yo;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotIntegerPlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImPlotPlotLine;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;

public class ImPlotYoPlot
{
   private final ImPlotPlot imPlotPlot;
   private final HashMap<YoVariable, ImPlotPlotLine> variablePlotLineMap = new HashMap<>();
   private final ArrayList<Pair<YoVariable, ImPlotPlotLine>> variablePlotLinePairList = new ArrayList<>();

   public ImPlotYoPlot(YoVariable... variablesToPlot)
   {
      this();
      addVariables(variablesToPlot);
   }

   public ImPlotYoPlot()
   {
      imPlotPlot = new ImPlotPlot();
   }

   public void addVariables(YoVariable... yoVariables)
   {
      for (YoVariable yoVariable : yoVariables)
      {
         addVariable(yoVariable);
      }
   }

   public void addVariable(YoVariable yoVariable)
   {
      ImPlotPlotLine plotLine = null;
      if (yoVariable instanceof YoDouble)
      {
         YoDouble yoDouble = (YoDouble) yoVariable;
         ImPlotDoublePlotLine doublePlotLine = new ImPlotDoublePlotLine(yoDouble.getName());
         plotLine = doublePlotLine;
         imPlotPlot.getPlotLines().add(doublePlotLine);
      }
      else if (yoVariable instanceof YoInteger)
      {
         YoInteger yoInteger = (YoInteger) yoVariable;
         ImPlotIntegerPlotLine integerPlotLine = new ImPlotIntegerPlotLine(yoInteger.getName());
         plotLine = integerPlotLine;
         imPlotPlot.getPlotLines().add(integerPlotLine);
      }
      variablePlotLineMap.put(yoVariable, plotLine);
      variablePlotLinePairList.add(ImmutablePair.of(yoVariable, plotLine));
   }

   public void removeVariable(YoVariable yoVariable)
   {
      imPlotPlot.getPlotLines().remove(variablePlotLineMap.get(yoVariable));
      variablePlotLineMap.remove(yoVariable);
      int indexToRemove = -1;
      for (int i = 0; i < variablePlotLinePairList.size(); i++)
      {
         Pair<YoVariable, ImPlotPlotLine> yoVariableImPlotPlotLinePair = variablePlotLinePairList.get(i);
         if (yoVariableImPlotPlotLinePair.getLeft().equals(yoVariable))
         {
            indexToRemove = i;
            break;
         }
      }
      variablePlotLinePairList.remove(indexToRemove);
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
         for (Pair<YoVariable, ImPlotPlotLine> yoVariableImPlotPlotLinePair : variablePlotLinePairList)
         {
            if (yoVariableImPlotPlotLinePair.getLeft() instanceof YoDouble)
            {
               YoDouble yoDouble = (YoDouble) yoVariableImPlotPlotLinePair.getLeft();
               ImPlotDoublePlotLine doublePlotLine = (ImPlotDoublePlotLine) yoVariableImPlotPlotLinePair.getRight();
               doublePlotLine.addValue(yoDouble.getValue());
            }
            else if (yoVariableImPlotPlotLinePair.getLeft() instanceof YoInteger)
            {
               YoInteger yoInteger = (YoInteger) yoVariableImPlotPlotLinePair.getLeft();
               ImPlotIntegerPlotLine integerPlotLine = (ImPlotIntegerPlotLine) yoVariableImPlotPlotLinePair.getRight();
               integerPlotLine.addValue(yoInteger.getValue());
            }
         }
      }
   }
}
