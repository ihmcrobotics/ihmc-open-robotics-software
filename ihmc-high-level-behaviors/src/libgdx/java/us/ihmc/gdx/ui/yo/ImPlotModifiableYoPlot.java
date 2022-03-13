package us.ihmc.gdx.ui.yo;

import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.simulation.scs2.GDXYoManager;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class ImPlotModifiableYoPlot
{
   private final ImPlotPlot imPlotPlot;
   private final HashMap<YoVariable, ImPlotPlotLine> variablePlotLineMap = new HashMap<>();
   private final ArrayList<Pair<YoVariable, ImPlotPlotLine>> variablePlotLinePairList = new ArrayList<>();
   private final ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel;
   private final ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final GDXYoManager yoManager;
   private boolean requestedVariable = false;
   private final Consumer<ImPlotModifiableYoPlot> removeSelf;

   public ImPlotModifiableYoPlot(ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel,
                                 ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel,
                                 GDXYoManager yoManager,
                                 Consumer<ImPlotModifiableYoPlot> removeSelf)
   {
      this.imGuiYoVariableSearchPanel = imGuiYoVariableSearchPanel;
      this.imPlotModifiableYoPlotPanel = imPlotModifiableYoPlotPanel;
      this.yoManager = yoManager;
      this.removeSelf = removeSelf;
      imPlotPlot = new ImPlotPlot();
      imPlotPlot.setPopupContextWindowImGuiRenderer(this::renderPopupContextWindow);
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
         LinkedYoVariable<YoDouble> linkedYoDoubleVariable = (LinkedYoVariable<YoDouble>) yoManager.newLinkedYoVariable(yoDouble);
         ImPlotYoBufferDoublePlotLine doublePlotLine = new ImPlotYoBufferDoublePlotLine(linkedYoDoubleVariable, this::removeVariable);
         plotLine = doublePlotLine;
         imPlotPlot.getPlotLines().add(doublePlotLine);
      }
      else if (yoVariable instanceof YoInteger)
      {
         YoInteger yoInteger = (YoInteger) yoVariable;
         LinkedYoVariable<YoInteger> linkedYoIntegerVariable = (LinkedYoVariable<YoInteger>) yoManager.newLinkedYoVariable(yoInteger);
         ImPlotYoBufferIntegerPlotLine integerPlotLine = new ImPlotYoBufferIntegerPlotLine(linkedYoIntegerVariable, this::removeVariable);
         plotLine = integerPlotLine;
         imPlotPlot.getPlotLines().add(integerPlotLine);
      }
      else
      {
         throw new RuntimeException("Implement for " + yoVariable.getClass().getSimpleName());
      }
      variablePlotLineMap.put(yoVariable, plotLine);
      variablePlotLinePairList.add(ImmutablePair.of(yoVariable, plotLine));
   }

   public void removeVariable(YoVariable yoVariable)
   {
      imPlotPlot.queueRemovePlotLine(variablePlotLineMap.get(yoVariable));
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

   public void render(float plotWidth, float plotHeight, boolean update)
   {
      if (requestedVariable && imGuiYoVariableSearchPanel.getSelectedVariable() != null)
      {
         requestedVariable = false;
         addVariable(imGuiYoVariableSearchPanel.getSelectedVariable());
         imGuiYoVariableSearchPanel.setSelectedVariable(null);
      }

      if (requestedVariable && !imGuiYoVariableSearchPanel.getSearchRequested())
      {
         requestedVariable = false; // Search was cancelled
      }

      updateVariables(update);
      imPlotPlot.render(plotWidth, plotHeight);
   }

   private void renderPopupContextWindow()
   {
      if (ImGui.menuItem(labels.get("Add Variable")))
      {
         imGuiYoVariableSearchPanel.setSearchRequested(true);
         requestedVariable = true;
         ImGui.closeCurrentPopup();
      }
      if (ImGui.menuItem(labels.get("Remove this plot")))
      {
         removeSelf.accept(this);
         ImGui.closeCurrentPopup();
      }
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

   public ArrayList<Pair<YoVariable, ImPlotPlotLine>> getVariablePlotLinePairList()
   {
      return variablePlotLinePairList;
   }
}
