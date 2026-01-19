package us.ihmc.rdx.ui.yo;

import imgui.internal.ImGui;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXYoManager;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.HashMap;

public class ImPlotModifiableYoPlot
{
   private final ImGuiSCSPlot imGuiSCSPlot;
   private final HashMap<YoVariable, ImGuiSCSPlotLine> variablePlotLineMap = new HashMap<>();
   private final ArrayList<Pair<YoVariable, ImGuiSCSPlotLine>> variablePlotLinePairList = new ArrayList<>();
   private final ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel;
   private final ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXYoManager yoManager;
   private boolean requestedVariable = false;

   public ImPlotModifiableYoPlot(ImGuiYoVariableSearchPanel imGuiYoVariableSearchPanel,
                                 ImPlotModifiableYoPlotPanel imPlotModifiableYoPlotPanel,
                                 RDXYoManager yoManager)
   {
      this.imGuiYoVariableSearchPanel = imGuiYoVariableSearchPanel;
      this.imPlotModifiableYoPlotPanel = imPlotModifiableYoPlotPanel;
      this.yoManager = yoManager;
      imGuiSCSPlot = new ImGuiSCSPlot();
      imGuiSCSPlot.setPopupContextWindowImGuiRenderer(this::renderPopupContextWindow);
   }

   public void addVariable(YoVariable yoVariable, boolean initializeLinkedVariable)
   {
      if (yoVariable == null)
      {
         LogTools.warn("YoVariable was null");
         return;
      }

      ImGuiSCSPlotLine plotLine = new ImGuiSCSPlotLine(yoVariable, this::removeVariable);
      if (initializeLinkedVariable)
         plotLine.setupLinkedVariable(yoManager);
      imGuiSCSPlot.getPlotLines().add(plotLine);
      variablePlotLineMap.put(yoVariable, plotLine);
      variablePlotLinePairList.add(ImmutablePair.of(yoVariable, plotLine));
   }

   public void removeVariable(YoVariable yoVariable)
   {
      imGuiSCSPlot.queueRemovePlotLine(variablePlotLineMap.get(yoVariable));
      variablePlotLineMap.remove(yoVariable);
      int indexToRemove = -1;
      for (int i = 0; i < variablePlotLinePairList.size(); i++)
      {
         Pair<YoVariable, ImGuiSCSPlotLine> yoVariableImPlotPlotLinePair = variablePlotLinePairList.get(i);
         if (yoVariableImPlotPlotLinePair.getLeft().equals(yoVariable))
         {
            indexToRemove = i;
            break;
         }
      }
      variablePlotLinePairList.remove(indexToRemove);
   }

   public void render(int column, int row, float plotWidth, float plotHeight)
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

      imGuiSCSPlot.render(column, row, plotWidth, plotHeight);
   }

   private void renderPopupContextWindow()
   {
      if (ImGui.menuItem(labels.get("Add Variable")))
      {
         imGuiYoVariableSearchPanel.setSearchRequested(true);
         requestedVariable = true;
         ImGui.closeCurrentPopup();
      }
   }

   public ArrayList<Pair<YoVariable, ImGuiSCSPlotLine>> getVariablePlotLinePairList()
   {
      return variablePlotLinePairList;
   }
}
