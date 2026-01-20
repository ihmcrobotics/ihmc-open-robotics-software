package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.flag.ImDrawFlags;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiHoveredFlags;
import imgui.flag.ImGuiMouseButton;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.MathTools;
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
   private boolean isDragging = false;

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
      while (!removalQueue.isEmpty())
         plotLines.remove(removalQueue.poll());

      float cursorX = ImGui.getCursorScreenPosX();
      float cursorY = ImGui.getCursorScreenPosY();

      ImGui.getWindowDrawList().addRect(cursorX + 1, cursorY + 1, cursorX + plotWidth, cursorY + plotHeight, ImGui.getColorU32(ImGuiCol.Border));

      ImGui.setCursorPos(ImGui.getCursorPosX() + 2, ImGui.getCursorPosY() + 2);

      float innerWidth = plotWidth - 2;
      float innerHeight = plotHeight - 3;
      cursorX = ImGui.getCursorScreenPosX();
      cursorY = ImGui.getCursorScreenPosY();
      int fontSize = ImGui.getFontSize();

      boolean isPlotHovered = ImGui.getMousePosX() >= cursorX
                           && ImGui.getMousePosX() <= cursorX + innerWidth
                           && ImGui.getMousePosY() >= cursorY
                           && ImGui.getMousePosY() <= cursorY + innerHeight;

      if (ImGui.isWindowHovered(ImGuiHoveredFlags.AllowWhenBlockedByActiveItem) && isPlotHovered && ImGui.isMouseReleased(ImGuiMouseButton.Left))
         addVariable(searchPanel.getSelectedVariable(), true);

      for (RDXSCS2PlotLine plotLine : plotLines)
         plotLine.updateData();

      double minValue = Double.POSITIVE_INFINITY;
      double maxValue = Double.NEGATIVE_INFINITY;
      for (RDXSCS2PlotLine plotLine : plotLines)
         if (plotLine.data != null)
         {
            minValue = Math.min(minValue, plotLine.minValue);
            maxValue = Math.max(maxValue, plotLine.maxValue);
         }
      double range = maxValue - minValue;

      if (ImGui.isWindowHovered() && isPlotHovered && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         isDragging = true;
      if (!ImGui.isMouseDown(ImGuiMouseButton.Left))
         isDragging = false;

      if (isDragging)
      {
         float dragXPlot = (float) MathTools.clamp(ImGui.getMousePosX(), cursorX, cursorX + innerWidth);
         yoManager.getSession().submitBufferIndexRequest(Math.round((dragXPlot - cursorX) * yoManager.getActiveBufferLength() / innerWidth));
      }

      float legendTextX = cursorX + 0.05f * fontSize;
      int numberLegendLines = 1;
      for (int lineIndex = 0; lineIndex < plotLines.size(); lineIndex++)
      {
         RDXSCS2PlotLine plotLine = plotLines.get(lineIndex);
         if (plotLine.data == null)
            continue;

         if (lineIndex < plotLines.size() - 1)
         {
            legendTextX += plotLine.legendTextSize + 0.2f * fontSize;
            if (legendTextX + plotLines.get(lineIndex + 1).legendTextSize > cursorX + innerWidth)
               ++numberLegendLines;
         }
      }

      float lineAreaHeight = innerHeight - (1.1f * numberLegendLines) * fontSize;

      int currentIndex = yoManager.getCurrentIndex();
      float verticalLineX = cursorX + currentIndex * innerWidth / yoManager.getActiveBufferLength();
      ImGui.getWindowDrawList().addLine(verticalLineX, cursorY, verticalLineX, cursorY + lineAreaHeight, ImGui.getColorU32(ImGuiCol.Text));

      if (minValue <= 0.0 && maxValue >= 0.0)
      {
         double normalizedZero = (0.0 - minValue) / range;
         float zeroLineY = cursorY + lineAreaHeight * (1.0f - (float) normalizedZero);
         float dashSize = 10.0f;
         float gapSize = 8.0f;
         float totalDashCycle = dashSize + gapSize;
         float currentX = cursorX;
         while (currentX < cursorX + innerWidth)
         {
            float dashEnd = Math.min(currentX + dashSize, cursorX + innerWidth);
            ImGui.getWindowDrawList().addLine(currentX, zeroLineY, dashEnd, zeroLineY, ImGui.getColorU32(ImGuiCol.Border), 2.0f);
            currentX += totalDashCycle;
         }
      }

      for (int lineIndex = 0; lineIndex < plotLines.size(); lineIndex++)
      {
         RDXSCS2PlotLine plotLine = plotLines.get(lineIndex);
         if (plotLine.data == null)
            continue;

         for (int i = 0; i < plotLine.points.length; i++)
         {
            float x = cursorX + i * innerWidth / plotLine.data.length;
            double normalized = (plotLine.data[i] - minValue) / range;
            float y = cursorY + lineAreaHeight * (1.0f - (float) normalized);
            plotLine.points[i].set(x, y);
         }

         int color = CHART_COLORS[lineIndex % CHART_COLORS.length];
         int imDrawFlags = ImDrawFlags.None;
         float thickness = 1.0f;
         ImGui.getWindowDrawList().addPolyline(plotLine.points, plotLine.points.length, color, imDrawFlags, thickness);
      }

      legendTextX = cursorX + 0.05f * fontSize;
      float legendTextY = cursorY  + innerHeight - (numberLegendLines + 0.05f) * fontSize;
      for (int lineIndex = 0; lineIndex < plotLines.size(); lineIndex++)
      {
         RDXSCS2PlotLine plotLine = plotLines.get(lineIndex);
         if (plotLine.data == null)
            continue;

         int color = CHART_COLORS[lineIndex % CHART_COLORS.length];
         boolean isHovered = ImGui.isMouseHoveringRect(legendTextX, legendTextY, legendTextX + plotLine.legendTextSize, legendTextY + fontSize);
         if (isHovered)
         {
            int r = (color) & 0xFF;
            int g = (color >> 8) & 0xFF;
            int b = (color >> 16) & 0xFF;
            int a = (int) (255 * 0.6f);
            color = (a << 24) | (b << 16) | (g << 8) | r;
         }
         ImGui.getWindowDrawList().addText(legendTextX, legendTextY, color, plotLine.legendText);

         if (isHovered && ImGui.isMouseClicked(ImGuiMouseButton.Middle))
            removalQueue.add(plotLine);

         if (lineIndex < plotLines.size() - 1)
         {
            legendTextX += plotLine.legendTextSize + 0.2f * fontSize;
            if (legendTextX + plotLines.get(lineIndex + 1).legendTextSize > cursorX + innerWidth)
            {
               legendTextX = cursorX + 0.05f * fontSize;
               legendTextY += 1.1f * fontSize;
            }
         }
      }

      ImGui.setCursorPos(ImGui.getCursorPosX() - 2, ImGui.getCursorPosY() - 2);

      ImGui.dummy(plotWidth, plotHeight);
   }

   private void renderPopupContextWindow()
   {
      if (ImGui.menuItem(labels.get("TODO")))
      {
         ImGui.closeCurrentPopup();
      }
   }

   public ArrayList<Pair<YoVariable, RDXSCS2PlotLine>> getVariablePlotLinePairList()
   {
      return variablePlotLinePairList;
   }
}
