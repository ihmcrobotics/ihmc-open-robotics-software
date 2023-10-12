package us.ihmc.rdx.ui.yo;

import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImPlotPlotLine;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

public class ImPlotModifiableYoPlot
{
   private final ImPlotPlot imPlotPlot;
   private final HashMap<YoVariable, ImPlotPlotLine> variablePlotLineMap = new HashMap<>();
   private final ArrayList<Pair<YoVariable, ImPlotYoBufferPlotLineBasics>> variablePlotLinePairList = new ArrayList<>();
   private final ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel;
   private final ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXYoManager yoManager;
   private boolean requestedVariable = false;
   private final Consumer<ImPlotModifiableYoPlot> removeSelf;

   public ImPlotModifiableYoPlot(ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel,
                                 ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel,
                                 RDXYoManager yoManager,
                                 Consumer<ImPlotModifiableYoPlot> removeSelf)
   {
      this.imGuiYoVariableSearchPanel = imGuiYoVariableSearchPanel;
      this.imPlotModifiableYoPlotPanel = imPlotModifiableYoPlotPanel;
      this.yoManager = yoManager;
      this.removeSelf = removeSelf;
      imPlotPlot = new ImPlotPlot();
      imPlotPlot.setPopupContextWindowImGuiRenderer(this::renderPopupContextWindow);
   }

   public void addVariable(YoVariable yoVariable, boolean initializeLinkedVariable)
   {
      ImPlotYoBufferPlotLineBasics plotLine = null;
      if (yoVariable instanceof YoDouble)
      {
         YoDouble yoDouble = (YoDouble) yoVariable;
         ImPlotYoBufferDoublePlotLine doublePlotLine = new ImPlotYoBufferDoublePlotLine(yoDouble, this::removeVariable);
         if (initializeLinkedVariable)
            doublePlotLine.setupLinkedVariable(yoManager);
         plotLine = doublePlotLine;
         imPlotPlot.getPlotLines().add(doublePlotLine);
      }
      else if (yoVariable instanceof YoInteger)
      {
         YoInteger yoInteger = (YoInteger) yoVariable;
         ImPlotYoBufferIntegerPlotLine integerPlotLine = new ImPlotYoBufferIntegerPlotLine(yoInteger, this::removeVariable);
         if (initializeLinkedVariable)
            integerPlotLine.setupLinkedVariable(yoManager);
         plotLine = integerPlotLine;
         imPlotPlot.getPlotLines().add(integerPlotLine);
      }
      else if (yoVariable instanceof YoLong)
      {
         YoLong yoInteger = (YoLong) yoVariable;
         ImPlotYoBufferLongPlotLine longPlotLine = new ImPlotYoBufferLongPlotLine(yoInteger, this::removeVariable);
         if (initializeLinkedVariable)
            longPlotLine.setupLinkedVariable(yoManager);
         plotLine = longPlotLine;
         imPlotPlot.getPlotLines().add(longPlotLine);
      }
      else if (yoVariable instanceof YoEnum)
      {
         YoEnum yoEnum = (YoEnum) yoVariable;
         ImPlotYoBufferEnumPlotLine enumPlotLine = new ImPlotYoBufferEnumPlotLine(yoEnum, this::removeVariable);
         if (initializeLinkedVariable)
            enumPlotLine.setupLinkedVariable(yoManager);
         plotLine = enumPlotLine;
         imPlotPlot.getPlotLines().add(enumPlotLine);
      }
      else if (yoVariable instanceof YoBoolean)
      {
         YoBoolean yoBoolean = (YoBoolean) yoVariable;
         ImPlotYoBufferBooleanPlotLine booleanPlotLine = new ImPlotYoBufferBooleanPlotLine(yoBoolean, this::removeVariable);
         if (initializeLinkedVariable)
            booleanPlotLine.setupLinkedVariable(yoManager);
         plotLine = booleanPlotLine;
         imPlotPlot.getPlotLines().add(booleanPlotLine);
      }
      else if (yoVariable == null)
      {
         LogTools.warn("YoVariable was null");
         return;
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
         Pair<YoVariable, ImPlotYoBufferPlotLineBasics> yoVariableImPlotPlotLinePair = variablePlotLinePairList.get(i);
         if (yoVariableImPlotPlotLinePair.getLeft().equals(yoVariable))
         {
            indexToRemove = i;
            break;
         }
      }
      variablePlotLinePairList.remove(indexToRemove);
   }

   public void render(float plotWidth, float plotHeight)
   {
      if (requestedVariable && imGuiYoVariableSearchPanel.getSelectedVariable() != null)
      {
         requestedVariable = false;
         addVariable(imGuiYoVariableSearchPanel.getSelectedVariable(), true);
         imGuiYoVariableSearchPanel.setSelectedVariable(null);
      }

      if (requestedVariable && !imGuiYoVariableSearchPanel.getSearchRequested())
      {
         requestedVariable = false; // Search was cancelled
      }

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

   public ArrayList<Pair<YoVariable, ImPlotYoBufferPlotLineBasics>> getVariablePlotLinePairList()
   {
      return variablePlotLinePairList;
   }
}
