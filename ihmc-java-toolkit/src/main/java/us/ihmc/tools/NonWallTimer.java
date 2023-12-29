package us.ihmc.tools;

public class NonWallTimer
{
   private double now = Double.NaN;
   private double resetTime = Double.NaN;

   public void update(double now)
   {
      this.now = now;
   }

   public void reset()
   {
      resetTime = now;
   }

   public double getElapsedTime()
   {
      return now - resetTime;
   }

   public boolean hasBeenSet()
   {
      return !Double.isNaN(resetTime);
   }

   public boolean isExpired(double expirationTime)
   {
      return hasBeenSet() && getElapsedTime() > expirationTime;
   }

   public boolean isRunning(double expirationTime)
   {
      return hasBeenSet() && !isExpired(expirationTime);
   }
}
