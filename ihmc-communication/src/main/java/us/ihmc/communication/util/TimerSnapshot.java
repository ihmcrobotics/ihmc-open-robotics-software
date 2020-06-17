package us.ihmc.communication.util;

public class TimerSnapshot
{
   private final double timePassedSinceReset;
   private final double time;

   public TimerSnapshot(double timePassedSinceReset, double time)
   {
      this.time = time;
      this.timePassedSinceReset = timePassedSinceReset;
   }

   public boolean isExpired()
   {
      return Timer.isExpired(timePassedSinceReset, time);
   }

   public boolean hasBeenSet()
   {
      return Timer.hasBeenSet(timePassedSinceReset);
   }

   public double getTimePassedSinceReset()
   {
      return timePassedSinceReset;
   }

   public boolean isRunning()
   {
      return hasBeenSet() && !isExpired();
   }
}
