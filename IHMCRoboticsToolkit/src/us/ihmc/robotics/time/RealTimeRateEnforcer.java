package us.ihmc.robotics.time;

/**
 * Class for enforcing that a simulation or other computational process doesn't run any faster than real time.
 * Of course, if it runs slower than real time, nothing you can do about it except speed up the algorithm
 * or buy a faster computer. 
 * 
 * Make sure to call reset() whenever the simulated clock and the wall clock become out of synch (for example, 
 * stopping or rewinding a simulation). 
 * 
 */
public class RealTimeRateEnforcer
{
   private long wallStartTimeInMilliseconds = -1;
   private double simulatedStartTimeInSeconds = -1.0;

   public void sleepIfNecessaryToEnforceRealTimeRate(double simulatedCurrentTimeInSeconds)
   {
      if (wallStartTimeInMilliseconds == -1)
      {
         wallStartTimeInMilliseconds = System.currentTimeMillis();
         simulatedStartTimeInSeconds = simulatedCurrentTimeInSeconds;
         return;
      }

      long wallCurrentTimeInMilliseconds = System.currentTimeMillis();

      int simulatedElapsedTimeInMilliseconds = (int) (1000.0 * (simulatedCurrentTimeInSeconds - simulatedStartTimeInSeconds));
      long wallElapsedTimeInMilliseconds = wallCurrentTimeInMilliseconds - wallStartTimeInMilliseconds;

      int timeToSleepInMilliseconds = (int) (simulatedElapsedTimeInMilliseconds - wallElapsedTimeInMilliseconds);
      if (timeToSleepInMilliseconds > 10)
      {
         try
         {
            Thread.sleep(timeToSleepInMilliseconds);
         }
         catch (InterruptedException e)
         {
         }
      }
   }

   public void reset()
   {
      wallStartTimeInMilliseconds = -1;
      simulatedStartTimeInSeconds = -1.0;
   }
}
