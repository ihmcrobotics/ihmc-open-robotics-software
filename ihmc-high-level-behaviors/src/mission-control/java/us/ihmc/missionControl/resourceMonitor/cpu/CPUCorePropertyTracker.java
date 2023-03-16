package us.ihmc.missionControl.resourceMonitor.cpu;

import java.util.ArrayDeque;

public class CPUCorePropertyTracker
{
   private final ArrayDeque<Double> values = new ArrayDeque<>();

   public void addIncrement(int historySize, double newValue)
   {
      values.addFirst(newValue);

      while (values.size() > historySize)
      {
         values.removeLast();
      }
   }

   public double getAverage()
   {
      if (values.isEmpty())
         return 0.0;

      double sum = 0.0;
      double lastValue = Double.NaN;
      for (Double value : values)
      {
         if (!Double.isNaN(lastValue))
         {
            sum += value - lastValue;
         }
         lastValue = value;
      }
      return sum / values.size();
   }
}
