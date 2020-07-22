package us.ihmc.communication.util;

public class TimerSnapshotWithExpiration extends TimerSnapshot
{
   private final double expirationTime;

   public TimerSnapshotWithExpiration(double timePassedSinceReset, double expirationTime)
   {
      super(timePassedSinceReset);
      this.expirationTime = expirationTime;
   }

   public boolean isExpired()
   {
      return isExpired(expirationTime);
   }

   public boolean isRunning()
   {
      return isRunning(expirationTime);
   }
}
