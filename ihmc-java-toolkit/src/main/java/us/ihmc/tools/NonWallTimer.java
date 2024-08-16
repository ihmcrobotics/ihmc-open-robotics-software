package us.ihmc.tools;

/**
 * A timer that uses an external time source that does not
 * have to be based on wall time or even increasing at a contant rate.
 * External time is submitted using the {@link #update} method.
 * For example, this is used to time things in a slower than
 * realtime simulations.
 */
public class NonWallTimer
{
   /** The current time in seconds. */
   private double now = Double.NaN;
   /** The latest time in which this timer was reset. */
   private double resetTime = Double.NaN;

   /** Submit external time here to be used by the other calculations. */
   public void update(double now)
   {
      this.now = now;
   }

   /** Resets or "sets" the timer to start ticking. */
   public void reset()
   {
      resetTime = now;
   }

   /** @return Time since the last reset or NaN if this timer has never been reset. */
   public double getElapsedTime()
   {
      return now - resetTime;
   }

   /** @return Whether this timer has ever been reset. */
   public boolean hasBeenReset()
   {
      return !Double.isNaN(resetTime);
   }

   /**
    * Unlike a real timer, the time of expiration is not set beforehand,
    * This method queries whether or not some time has passed.
    *
    * @return expired or false if the timer has never been reset
    */
   public boolean isExpired(double expirationTime)
   {
      return hasBeenReset() && getElapsedTime() > expirationTime;
   }

   /**
    * @return If the timer would be ticking if it was a real timer
    *         reset with given expiration time, of false if this timer
    *         has never been reset.
    */
   public boolean isRunning(double expirationTime)
   {
      return hasBeenReset() && !isExpired(expirationTime);
   }
}
