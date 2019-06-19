package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class ThreadTimer
{
   private final long expectedDTNanos;
   private final double alpha;

   private final YoLong tick;
   private final YoDouble dt;
   private final YoDouble timer;
   private final YoLong jitter;
   private final YoDouble realtimeRate;

   private long lastStartTime;

   /**
    * Creates a periodic thread timer including Jitter estimation.
    *
    * @param name prefix for the timing variables in the registry
    * @param expectedDt of the periodic execution for Jitter estimation in seconds
    * @param registry to attach timing variables to
    */
   public ThreadTimer(String name, double expectedDt, YoVariableRegistry registry)
   {
      expectedDTNanos = Conversions.secondsToNanoseconds(expectedDt);
      alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, Conversions.nanosecondsToSeconds(expectedDTNanos));

      tick = new YoLong(name + "Tick", registry);
      dt = new YoDouble(name + "DT", registry);
      timer = new YoDouble(name + "Timer", registry);
      jitter = new YoLong(name + "JitterInNanos", registry);
      realtimeRate = new YoDouble(name + "RealtimeRate", registry);

      tick.set(-1);
      realtimeRate.set(1.0);
   }

   /**
    * Creates a periodic thread timer without Jitter estimation.
    *
    * @param name prefix for the timing variables in the registry
    * @param registry to attach timing variables to
    */
   public ThreadTimer(String name, YoVariableRegistry registry)
   {
      expectedDTNanos = 0;
      alpha = 0.0;
      jitter = null;
      realtimeRate = null;

      tick = new YoLong(name + "Tick", registry);
      dt = new YoDouble(name + "DT", registry);
      timer = new YoDouble(name + "Timer", registry);

      tick.set(-1);
   }

   public void start()
   {
      long startTime = System.nanoTime();
      if (lastStartTime != 0)
      {
         computeJitter(startTime);
         double dtInNanos = startTime - lastStartTime;
         dt.set(Conversions.nanosecondsToMilliseconds(dtInNanos));
         computeRealtimeRate(dtInNanos);
      }
      lastStartTime = startTime;
      tick.increment();
   }

   private void computeRealtimeRate(double dtInNanos)
   {
      if (realtimeRate == null)
         return;

      double newRate = expectedDTNanos / dtInNanos;
      realtimeRate.set(alpha * realtimeRate.getValue() + (1.0 - alpha) * newRate);
   }

   private void computeJitter(long startTime)
   {
      if (jitter == null)
         return;

      // Jitter computation as done in the EtherCAT master.
      // As suggested in RFC1889 the estimate is filtered using a magic number.
      long delta = startTime - lastStartTime - expectedDTNanos;
      if (delta < 0)
         delta = -delta;
      jitter.add((delta - jitter.getValue()) / 16);
   }

   public long getTickCount()
   {
      return tick.getValue();
   }

   public void stop()
   {
      timer.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - lastStartTime)));
   }
}
