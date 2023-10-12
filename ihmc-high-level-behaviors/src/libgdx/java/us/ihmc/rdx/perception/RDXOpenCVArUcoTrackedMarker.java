package us.ihmc.rdx.perception;

import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;

public class RDXOpenCVArUcoTrackedMarker
{
   private final int id;
   private boolean currentlyDetected = false;
   private final ImPlotPlot detectedPlot = new ImPlotPlot(30);
   private final ImPlotDoublePlotLine detectedPlotLine = new ImPlotDoublePlotLine("Detected");

   public RDXOpenCVArUcoTrackedMarker(int id)
   {
      this.id = id;
      detectedPlot.getPlotLines().add(detectedPlotLine);
   }

   public int getId()
   {
      return id;
   }

   public void setCurrentlyDetected(boolean currentlyDetected)
   {
      this.currentlyDetected = currentlyDetected;
   }

   public boolean getCurrentlyDetected()
   {
      return currentlyDetected;
   }

   public void renderPlotLine()
   {
      detectedPlotLine.addValue(currentlyDetected ? 1.0 : 0.0);
      detectedPlot.render();
   }
}
