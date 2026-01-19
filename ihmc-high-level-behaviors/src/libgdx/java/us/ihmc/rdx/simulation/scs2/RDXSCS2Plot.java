package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXSCS2Plot
{
   public static final int[] CHART_COLORS = new int[]
   {
      ImGui.getColorU32(160.0f / 255.0f, 0.0f, 0.0f, 1.0f),
      ImGui.getColorU32(0.0f, 0.0f, 1.0f, 1.0f),
      ImGui.getColorU32(0.0f, 128.0f / 255.0f, 0.0f, 1.0f),
      ImGui.getColorU32(0.0f, 0.0f, 0.0f, 1.0f),
      ImGui.getColorU32(128.0f / 255.0f, 128.0f / 255.0f, 128.0f / 255.0f, 1.0f),
      ImGui.getColorU32(128.0f / 255.0f, 0.0f, 128.0f / 255.0f, 1.0f),
      ImGui.getColorU32(0.0f, 128.0f / 255.0f, 128.0f / 255.0f, 1.0f),
      ImGui.getColorU32(96.0f / 255.0f, 96.0f / 255.0f, 0.0f, 1.0f),
      ImGui.getColorU32(1.0f, 80.0f / 255.0f, 80.0f / 255.0f, 1.0f),
      ImGui.getColorU32(80.0f / 255.0f, 1.0f, 1.0f, 1.0f)
   };

   private final ArrayList<RDXSCS2PlotLine> plotLines = new ArrayList<>();
   private final ConcurrentLinkedQueue<RDXSCS2PlotLine> removalQueue = new ConcurrentLinkedQueue<>();
   private final HashMap<YoVariable, RDXSCS2PlotLine> variablePlotLineMap = new HashMap<>();
   private final ArrayList<Pair<YoVariable, RDXSCS2PlotLine>> variablePlotLinePairList = new ArrayList<>();
   private final RDXSCS2YoVariableSearchPanel searchPanel;
   private final RDXSCS2PlotPanel panel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RDXYoManager yoManager;
   private boolean requestedVariable = false;

   public RDXSCS2Plot(RDXSCS2YoVariableSearchPanel searchPanel, RDXSCS2PlotPanel panel, RDXYoManager yoManager)
   {
      this.searchPanel = searchPanel;
      this.panel = panel;
      this.yoManager = yoManager;
   }

   public void addVariable(YoVariable yoVariable, boolean initializeLinkedVariable)
   {
      if (yoVariable == null)
      {
         LogTools.warn("YoVariable was null");
         return;
      }

      RDXSCS2PlotLine plotLine = new RDXSCS2PlotLine(yoVariable, this::removeVariable);
      if (initializeLinkedVariable)
         plotLine.setupLinkedVariable(yoManager);
      plotLines.add(plotLine);
      variablePlotLineMap.put(yoVariable, plotLine);
      variablePlotLinePairList.add(ImmutablePair.of(yoVariable, plotLine));
   }

   public void removeVariable(YoVariable yoVariable)
   {
      removalQueue.add(variablePlotLineMap.get(yoVariable));
      variablePlotLineMap.remove(yoVariable);
      int indexToRemove = -1;
      for (int i = 0; i < variablePlotLinePairList.size(); i++)
      {
         Pair<YoVariable, RDXSCS2PlotLine> yoVariableImPlotPlotLinePair = variablePlotLinePairList.get(i);
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
      if (requestedVariable && searchPanel.getSelectedVariable() != null)
      {
         requestedVariable = false;
         addVariable(searchPanel.getSelectedVariable(), true);
         searchPanel.setSelectedVariable(null);
      }

      if (requestedVariable && !searchPanel.getSearchRequested())
      {
         requestedVariable = false; // Search was cancelled
      }

      while (!removalQueue.isEmpty())
         plotLines.remove(removalQueue.poll());

      float cursorX = ImGui.getCursorScreenPosX();
      float cursorY = ImGui.getCursorScreenPosY();

      ImGui.getWindowDrawList().addRect(cursorX + 1, cursorY + 1, cursorX + plotWidth, cursorY + plotHeight, ImGui.getColorU32(ImGuiCol.Border));

      ImGui.setCursorPos(ImGui.getCursorPosX() + 2, ImGui.getCursorPosY() + 2);

      for (int i = 0; i < plotLines.size(); i++)
         plotLines.get(i).render(plotWidth - 2, plotHeight - 3, i);

      ImGui.setCursorPos(ImGui.getCursorPosX() - 2, ImGui.getCursorPosY() - 2);

      ImGui.dummy(plotWidth, plotHeight);
   }

   private void renderPopupContextWindow()
   {
      if (ImGui.menuItem(labels.get("Add Variable")))
      {
         searchPanel.setSearchRequested(true);
         requestedVariable = true;
         ImGui.closeCurrentPopup();
      }
   }

   public ArrayList<Pair<YoVariable, RDXSCS2PlotLine>> getVariablePlotLinePairList()
   {
      return variablePlotLinePairList;
   }
}
