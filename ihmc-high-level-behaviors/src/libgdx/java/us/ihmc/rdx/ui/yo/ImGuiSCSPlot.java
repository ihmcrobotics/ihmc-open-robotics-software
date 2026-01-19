package us.ihmc.rdx.ui.yo;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiTools;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ImGuiSCSPlot
{
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
      ImGui.getWindowDrawList().addText(cursorX, cursorY, ImGuiTools.BLACK, "Plot " + column + ", " + row);

      for (ImGuiSCSPlotLine plotLine : plotLines)
         plotLine.render(plotWidth, plotHeight);

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
