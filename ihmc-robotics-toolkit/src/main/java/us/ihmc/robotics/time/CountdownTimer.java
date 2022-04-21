package us.ihmc.robotics.time;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

// All information needed to set a timer in a single class.
// Set a time, and the timer will "timeOut" when that much time has elapsed

public class CountdownTimer
{
   private final YoDouble maxTime;
   private final YoDouble timeLeft;
   private final YoBoolean isStopped;

   private double startTime;

   public CountdownTimer(String name, YoRegistry registry)
   {
      this(name, 0.0, registry);
   }

   public CountdownTimer(String name, double maxTime, YoRegistry registry)
   {
      this.maxTime = new YoDouble(name + "MaxTime", registry);
      this.maxTime.set(maxTime);

      this.timeLeft = new YoDouble(name + "TimeLeft", registry);
      this.timeLeft.set(0.0);

      isStopped = new YoBoolean(name + "IsStopped", registry);
      isStopped.set(true);

      startTime = 0.0;
   }

   public void restartCountdown()
   {
      startCountdown(maxTime.getDoubleValue());
   }

   public void startCountdown(double time)
   {
      startTime = getCurrentAbsoluteTime();
      maxTime.set(time);
      isStopped.set(false);
   }

   // This is equivalent to an update loop
   public boolean isRinging()
   {
      if (!isStopped.getBooleanValue() || checkTimeLeft() <= 0.0)
      {
         return true;
      }
      return false;
   }

   // Prevents the timer from going off again. Useful for showing that the timer has been "checked"
   public void stopRinging()
   {
      isStopped.set(true);
   }

   public boolean isStopped()
   {
      return isStopped.getBooleanValue();
   }

   public double checkTimeLeft()
   {
      timeLeft.set((startTime + maxTime.getDoubleValue()) - getCurrentAbsoluteTime());
      return timeLeft.getDoubleValue();
   }

   private double getCurrentAbsoluteTime()
   {
      return Conversions.nanosecondsToSeconds(System.nanoTime());
   }
}
