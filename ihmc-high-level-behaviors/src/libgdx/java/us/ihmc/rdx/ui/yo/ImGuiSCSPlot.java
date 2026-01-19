package us.ihmc.rdx.ui.yo;

import imgui.ImGui;
import imgui.flag.ImGuiCol;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ImGuiSCSPlot
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

   private final ArrayList<ImGuiSCSPlotLine> plotLines = new ArrayList<>();
   private final ConcurrentLinkedQueue<ImGuiSCSPlotLine> removalQueue = new ConcurrentLinkedQueue<>();

   public ImGuiSCSPlot()
   {

   }

   public void render(int column, int row, float plotWidth, float plotHeight)
   {
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

   public void setPopupContextWindowImGuiRenderer(Runnable renderPopupContextWindow)
   {

   }

   public void queueRemovePlotLine(ImGuiSCSPlotLine plotLineToRemove)
   {
      removalQueue.add(plotLineToRemove);
   }

   public ArrayList<ImGuiSCSPlotLine> getPlotLines()
   {
      return plotLines;
   }
}
