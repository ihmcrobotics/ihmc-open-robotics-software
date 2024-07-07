package us.ihmc.tools;

public class TimerSnapshotWithExpiration extends TimerSnapshot
{
   private final double expirationDuration;

   public TimerSnapshotWithExpiration(double timePassedSinceReset, double expirationDuration)
   {
      super(timePassedSinceReset);
      this.expirationDuration = expirationDuration;
   }

   public boolean isExpired()
   {
      return isExpired(expirationDuration);
   }

   public boolean isRunning()
   {
      return isRunning(expirationDuration);
   }
}
