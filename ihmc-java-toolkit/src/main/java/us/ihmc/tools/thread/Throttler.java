package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.tools.Timer;

public class Throttler
{
   private final Timer timer = new Timer();
   private double optionallySetPeriod = Double.NaN;

   /**
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
    * For use if the user set the period with the setPeriod method.
    */
   public boolean run()
   {
      return run(optionallySetPeriod);
   }

   /**
    * Bring your own period, especially if it is dynamically calculated.
    */
   public boolean run(double period)
   {
      boolean run = !timer.isRunning(period);
      if (run)
      {
         timer.reset();
      }
      return run;
   }

   /**
    * For use if the user set the period with the setPeriod method.
    */
   public void waitAndRun()
   {
      waitAndRun(optionallySetPeriod);
   }

   /**
    * Bring your own period, especially if it is dynamically calculated.
    */
   public void waitAndRun(double period)
   {
      timer.sleepUntilExpiration(period);
      timer.reset();
   }
}
