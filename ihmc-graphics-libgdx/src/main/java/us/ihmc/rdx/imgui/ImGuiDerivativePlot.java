package us.ihmc.rdx.imgui;

import us.ihmc.commons.time.Stopwatch;

public class ImGuiDerivativePlot extends ImGuiPlot
{
   private float previousValue = Float.NaN;
   private final Stopwatch stopwatch = new Stopwatch();

   public ImGuiDerivativePlot(String name, int bufferSize)
   {
      super(name, bufferSize);
   }

   public ImGuiDerivativePlot(String name, int bufferSize, int width, int height)
   {
      super(name, bufferSize, width, height);
   }

   @Override
   public void setValue(float newValue)
   {
      if (Float.isNaN(previousValue))
      {
         super.setValue(Float.NaN);
         stopwatch.reset();
      }
      else
      {
         float lap = (float) stopwatch.lap();
         super.setValue((newValue - previousValue) / lap);
      }

      previousValue = newValue;
   }
}
