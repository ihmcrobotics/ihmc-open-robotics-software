package us.ihmc.tools;

import us.ihmc.commons.time.Stopwatch;

/**
 * This class is like Timer but has a "turn off" functionality and different naming.
 * Also saves the maximum time so that reset can be called more conveniently
 * This class probably belongs somewhere else
 *
 * This class represents a kitchen timer.
 * A timer may be in one of the states "off", "ticking down", and "ringing"
 * The timer can be "started" and begin "ticking down"
 * Once the maximum time has elapsed, it will start "ringing"
 * It will continue ringing until the timer is turned "off" or it is "restarted"
 */
public class KitchenTimer
{
   private final Stopwatch stopwatch = new Stopwatch();
   private double maximumTime;
   private boolean isOff = true;

   public KitchenTimer(double maximumTime)
   {
      this.maximumTime = maximumTime;
   }

   public void setAndStart(double maximumTime)
   {
      this.maximumTime = maximumTime;
      restart();
   }

   /**
    * Reset or "crank" the timer back to zero time elapsed.
    */
   public void restart()
   {
      isOff = false;
      stopwatch.start();
   }

   public void turnOff()
   {
      isOff = true;
   }

   public boolean timerIsOff()
   {
      return isOff;
   }

   public boolean isRinging()
   {
      return (!timerIsOff()) && (getElapsedTime() > maximumTime);
   }

   public boolean isTickingDown()
   {
      return (!timerIsOff() && (getElapsedTime() < maximumTime));
   }

   /**
    * @return Total elapsed time.
    */
   public double getElapsedTime()
   {
      return stopwatch.totalElapsed();
   }
}
