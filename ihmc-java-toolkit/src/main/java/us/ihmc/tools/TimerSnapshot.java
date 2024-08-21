package us.ihmc.tools;

public class TimerSnapshot
{
   private final double timePassedSinceReset;

   public TimerSnapshot(double timePassedSinceReset)
   {
      this.timePassedSinceReset = timePassedSinceReset;
   }

   public boolean isRunning(double expirationDuration)
   {
      return Timer.isRunning(timePassedSinceReset, expirationDuration);
   }

   public boolean isExpired(double expirationDuration)
   {
      return Timer.isExpired(timePassedSinceReset, expirationDuration);
   }

   public boolean hasBeenSet()
   {
      return Timer.hasBeenSet(timePassedSinceReset);
   }

   public double getTimePassedSinceReset()
   {
      return timePassedSinceReset;
   }

   public TimerSnapshotWithExpiration withExpiration(double expirationDuration)
   {
      return new TimerSnapshotWithExpiration(timePassedSinceReset, expirationDuration);
   }
}
