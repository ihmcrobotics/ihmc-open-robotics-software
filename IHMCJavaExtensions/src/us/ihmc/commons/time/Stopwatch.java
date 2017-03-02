package us.ihmc.commons.time;

/**
 * <p>Different from {@link org.apache.commons.lang3.time.StopWatch StopWatch} in
 * Apache Commons Lang in the following ways:</p>
 * 
 * <li>Provides a more fluent interface</li>
 * <li>Keeps track of average lap</li>
 * 
 * <p>TODO: Add suspend and resume.</p>
 */
public class Stopwatch
{
   private double currentTime;
   private double lastTime;
   private double deltaTime;
   private double startTime;
   private long numLaps;
   private double deltaSum;
   
   /**
    * Start the clock. Functionally the same as reset.
    * 
    * @return <code>this</code> for convenience.
    */
   public Stopwatch start()
   {
      reset();
      
      return this;
   }
   
   /**
    * Reset the current lap. Does not get averaged.
    */
   public void resetLap()
   {
      lastTime = now();
   }

   /**
    * Reset the stopwatch. Clears average, resets lap, starts the clock.
    */
   public void reset()
   {
      startTime = now();
      lastTime = startTime;
      
      numLaps = 0;
      deltaSum = 0.0;
   }
   
   /**
    * Record a lap. Shortest time since last lap, start, or reset is returned and averaged.
    * 
    * @return Lap time.
    */
   public double lap()
   {
      currentTime = now();
      deltaTime = currentTime - lastTime;
      lastTime = currentTime;

      // for average lap
      numLaps++;
      deltaSum += deltaTime;
      
      return deltaTime;
   }
   
   /**
    * Get the average lap duration.
    * 
    * @return Average lap duration.
    */
   public double averageLap()
   {
      return deltaSum / numLaps;
   }
   
   /**
    * Get the total elapsed time. Since the last reset or start.
    * 
    * @return Total elapsed time.
    */
   public double totalElapsed()
   {
      return now() - startTime;
   }
   
   /**
    * Get the elapsed time in the current lap. Shortest time since last lap, reset, or start.
    * 
    * @return Lap elapsed time.
    */
   public double lapElapsed()
   {
      return now() - lastTime;
   }
   
   private double now()
   {
      return System.nanoTime() / 1e9;
   }
}
