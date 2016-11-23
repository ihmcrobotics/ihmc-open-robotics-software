package us.ihmc.robotics.time;

import java.util.ArrayList;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;

/**
 * @deprecated Use us.ihmc.tools.time.Timer or us.ihmc.simulationconstructionset.util.time.YoTimer
 */
public class GlobalTimer
{
   private static final boolean COMPUTE_MEAN = false;
   private static final ArrayList<GlobalTimer> allTimers = new ArrayList<GlobalTimer>();
   private static final long initialTimeNanos = System.nanoTime();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable timersAlpha;

   private final LongYoVariable startTime, stopTime;
   private final DoubleYoVariable duration, runningAvg;
   private final AlphaFilteredYoVariable filteredDuration;

   private final ArrayList<Double> pastNValues;

   private final String timerName;

   public GlobalTimer(String timerName, YoVariableRegistry parentRegistry)
   {
      this.timerName = timerName;
      registry = new YoVariableRegistry(timerName);
      timersAlpha = new DoubleYoVariable("timersAlpha", registry);
      timersAlpha.set(0.97);

      startTime = new LongYoVariable(timerName + "StartTimeNano", registry);
      stopTime = new LongYoVariable(timerName + "StopTimeNano", registry);
      duration = new DoubleYoVariable(timerName + "DurationMilli", registry);
      filteredDuration = new AlphaFilteredYoVariable(timerName + "FiltDurationMilli", registry, timersAlpha, duration);
      filteredDuration.update();

      if (COMPUTE_MEAN)
      {
         runningAvg = new DoubleYoVariable(timerName + "AvgDurationMilli", registry);
         pastNValues = new ArrayList<Double>();
      }

      else
      {
         runningAvg = null;
         pastNValues = null;
      }

      allTimers.add(this);
      parentRegistry.addChild(registry);
   }

   public static void clearTimers()
   {
      allTimers.clear();
   }

   public void startTimer()
   {
      startTime.set(System.nanoTime() - initialTimeNanos);
   }

   public void stopTimer()
   {
      stopTime.set(System.nanoTime() - initialTimeNanos);
      duration.set((stopTime.getLongValue() - startTime.getLongValue()) * 1e-6);
      filteredDuration.update();

      if (COMPUTE_MEAN)
      {
         if (pastNValues.size() > 100)
         {
            pastNValues.remove(0);
         }

         pastNValues.add(duration.getDoubleValue());
         runningAvg.set(MathTools.mean(pastNValues));
      }
   }

   public static String listNamesOfAverageDurations()
   {
      if (allTimers.size() == 0)
      {
         return "";
      }

      GlobalTimer timer = allTimers.get(0);
      String ret = timer.runningAvg.getName();
      for (int i = 1; i < allTimers.size(); i++)
      {
         timer = allTimers.get(i);
         ret += "\n" + timer.runningAvg.getName();
      }

      return ret;
   }

   public static String listAverageDurations()
   {
      if (allTimers.size() == 0)
      {
         return "";
      }

      GlobalTimer timer = allTimers.get(0);
      String ret = timer.runningAvg.getDoubleValue() + "";
      for (int i = 1; i < allTimers.size(); i++)
      {
         timer = allTimers.get(i);
         ret += "\n" + timer.runningAvg.getDoubleValue();
      }

      return ret;
   }

   public double getElapsedTime()
   {
      long currentSysTime = System.nanoTime() - initialTimeNanos;
      double currentTime = (currentSysTime - startTime.getLongValue()) * 1e-6;
      return currentTime;
   }

   public String getTimerName()
   {
      return timerName;
   }

   public static void getAlltimers(ArrayList<GlobalTimer> allTimersToPack)
   {
      allTimersToPack.addAll(allTimers);
   }

   public static boolean isComputeMean()
   {
      return COMPUTE_MEAN;
   }

   public String getYoDurationName()
   {
      return duration.getName();
   }

   public String getYoFilteredDurationName()
   {
      return filteredDuration.getName();
   }

   public String getYoRunningAvgName()
   {
      return runningAvg.getName();
   }
}
