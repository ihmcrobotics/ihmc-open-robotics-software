package us.ihmc.communication.util;

public class TimerSnapshot
{
   private final double timePassedSinceReset;

   public TimerSnapshot(double timePassedSinceReset)
   {
      this.timePassedSinceReset = timePassedSinceReset;
   }

   public boolean isExpired(double expirationTime)
   {
      return Timer.isExpired(timePassedSinceReset, expirationTime);
   }

   public boolean hasBeenSet()
   {
      return Timer.hasBeenSet(timePassedSinceReset);
   }

   public double getTimePassedSinceReset()
   {
      return timePassedSinceReset;
   }

   public boolean isRunning(double expirationTime)
   {
      return hasBeenSet() && !isExpired(expirationTime);
   }

   public TimerSnapshotWithExpiration withExpiration(double expirationTime)
   {
      return new TimerSnapshotWithExpiration(timePassedSinceReset, expirationTime);
   }
}
