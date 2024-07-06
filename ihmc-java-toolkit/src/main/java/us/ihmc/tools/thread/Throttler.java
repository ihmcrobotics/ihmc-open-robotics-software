package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;

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
    *
    * For use if the user set the period with the setPeriod method.
    */
   public boolean run()
   {
      return run(optionallySetPeriod);
   }

   /**
    * @return Whether or not enough time has passed to run your thing again.
    *
    * Bring your own period, especially if it is dynamically calculated.
    */
   public boolean run(double period)
   {
      double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (Double.isNaN(resetTime)) // First run
      {
         resetTime = currentTime;
         return true;
      }
      else
      {
         double elapsedTime = currentTime - resetTime;
         double overtime = elapsedTime - period;
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
    * For use if the user set the period with the setPeriod method.
    */
   public void waitAndRun()
   {
      waitAndRun(optionallySetPeriod);
   }

   /**
    * Sleeps until enough time has passed to run your thing again.
    *
    * Bring your own period, especially if it is dynamically calculated.
    */
   public void waitAndRun(double period)
   {
      double currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (Double.isNaN(resetTime)) // First run
      {
         resetTime = currentTime;
      }
      else
      {
         double elapsedTime = currentTime - resetTime;
         double overtime = elapsedTime - period;

         if (overtime < 0.0)
         {
            ThreadTools.sleepSeconds(-overtime); // Guarantees to sleep at least this amount (i.e. will sleep too long)

            // Measure the time again so we can set the reset time correctly
            currentTime = Conversions.nanosecondsToSeconds(System.nanoTime());
            elapsedTime = currentTime - resetTime;
            overtime = elapsedTime - period;
         }

         // We subtract the overtime to achieve a more accurate rate
         resetTime = currentTime - overtime;
      }
   }
}
