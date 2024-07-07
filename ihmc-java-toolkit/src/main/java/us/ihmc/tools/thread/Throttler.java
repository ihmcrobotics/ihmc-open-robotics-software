package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;

/**
 * Throttler is used to allow things to happen at a slower rate
 * than the parent update thread. It features a run method,
 * which returns whether or not enough time has passed to run something.
 * Alternatively, it features a waitAndRun method, which allows the
 * user to sleep until it is ready for the next thing.
 *
 * This class is useful as an alternative to creating additional
 * threads when appropriate. It does not create any threads so
 * there are no concurrency issues.
 *
 * Throttler's {@link #run} methods work best when polled at much higher frequencies
 * than the requested throttled frequency. We have put a mechanism in place
 * to handle poll frequencies near or lower than requested, but the result
 * will be jittery and inaccurate. In those cases, it is best to spin
 * up a separate thread. See {@link RestartableThrottledThread}.
 *
 * Throttler's {@link #waitAndRun} methods have a different nature. They
 * obviously cannot be called faster than the requested frequency. They
 * are designed to handle waiting the extra time after variable amounts
 * of computation in order to run that computation at a steady rate.
 *
 * Example:
 *
 * <pre>
 *    Throttler throttler = new Throttler().setFrequency(5.0);
 *
 *    while (true)
 *    {
 *       if (throttler.run())
 *       {
 *          // do stuff
 *       }
 *    }
 * </pre>
 */
public class Throttler
{
   private double resetTime = Double.NaN;
   private double optionallySetPeriod = Double.NaN;
   private transient double currentTime;
   private transient double overtime;

   /**
    * Set the period.
    *
    * Syntactic sugar to be clear about what a constant passed in would be.
    * For example, as a field:
    *
    * <pre>
    *    private final Throttler throttler = new Throttler().setPeriod(5.0);
    * </pre>
    */
   public Throttler setPeriod(double period)
   {
      optionallySetPeriod = period;
      return this;
   }

   /**
    * Set the frequency.
    *
    * Syntactic sugar to be clear about what a constant passed in would be.
    * For example, as a field:
    *
    * <pre>
    *    private final Throttler throttler = new Throttler().setFrequency(5.0);
    * </pre>
    */
   public Throttler setFrequency(double frequency)
   {
      optionallySetPeriod = Conversions.hertzToSeconds(frequency);
      return this;
   }

   /**
    * @return Whether or not enough time has passed to run your thing again.
    *         It is recommended to call this at several times the desired throttled rate.
    *         Calling this more often is directly propotional to the resulting accuracy.
    *
    * For use if the user set the period with the {@link #setPeriod} method.
    */
   public boolean run()
   {
      return run(optionallySetPeriod);
   }

   /**
    * @param period for passing in dynamically calculated periods
    * @return Whether or not enough time has passed to run your thing again.
    *         It is recommended to call this at several times the desired throttled rate.
    *         Calling this more often is directly propotional to the resulting accuracy.
    */
   public boolean run(double period)
   {
      currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (Double.isNaN(resetTime)) // First run
      {
         resetTime = currentTime;
         return true;
      }
      else
      {
         calculateOvertime(period);

         boolean periodHasElapsed = overtime >= 0.0;

         if (periodHasElapsed)
         {
            // We subtract the overtime to achieve a more accurate rate
            resetTime = currentTime - overtime;
         }

         return periodHasElapsed;
      }
   }

   /**
    * Sleeps until enough time has passed to run your thing again.
    *
    * For use if the user set the period with the {@link #setPeriod} method.
    */
   public void waitAndRun()
   {
      waitAndRun(optionallySetPeriod);
   }

   /**
    * Sleeps until enough time has passed to run your thing again.
    *
    * @param period for passing in dynamically calculated periods
    */
   public void waitAndRun(double period)
   {
      currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (Double.isNaN(resetTime)) // First run
      {
         resetTime = currentTime;
      }
      else
      {
         calculateOvertime(period);

         if (overtime < 0.0)
         {
            MissingThreadTools.sleepAtLeast(-overtime); // Guarantees to sleep at least this amount (i.e. will sleep too long)

            // Measure the time again so we can set the reset time correctly
            currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());
            calculateOvertime(period);
         }

         // We subtract the overtime to achieve a more accurate rate
         resetTime = currentTime - overtime;
      }
   }

   private void calculateOvertime(double period)
   {
      double elapsedTime = currentTime - resetTime;

      // Limit the overtime to half the period, to prevent building up
      // when the run methods are called too infrequently.
      overtime = Math.min(elapsedTime - period, 0.5 * period);
   }
}
