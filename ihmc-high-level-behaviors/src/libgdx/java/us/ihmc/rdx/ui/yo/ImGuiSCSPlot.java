package us.ihmc.rdx.ui.yo;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ImGuiSCSPlot
{
   private final ArrayList<ImGuiSCSPlotLine> plotLines = new ArrayList<>();
   private final ConcurrentLinkedQueue<ImGuiSCSPlotLine> removalQueue = new ConcurrentLinkedQueue<>();

   public ImGuiSCSPlot()
   {

   }

   public void render(float plotWidth, float plotHeight)
   {

      while (!removalQueue.isEmpty())
         plotLines.remove(removalQueue.poll());
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
