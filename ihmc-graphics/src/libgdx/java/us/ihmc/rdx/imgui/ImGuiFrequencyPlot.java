package us.ihmc.rdx.imgui;

import imgui.internal.ImGui;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.tools.UnitConversions;

public class ImGuiFrequencyPlot
{
   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("Hz", 1000, 230, 20);
   private final Stopwatch stopwatch = new Stopwatch().start();
   private volatile double elapsedSinceReceived = Double.NaN;

   public void recordEvent()
   {
      ++receivedCount;
      elapsedSinceReceived = stopwatch.lap();
   }

   public void renderImGuiWidgets()
   {
      receivedPlot.render((float) UnitConversions.secondsToHertz(elapsedSinceReceived));
   }

   public boolean anyEventsYet()
   {
      return receivedCount > 0;
   }

   public ImGuiPlot getReceivedPlot()
   {
      return receivedPlot;
   }
}
