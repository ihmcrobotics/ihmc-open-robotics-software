package us.ihmc.commons.time;

import java.util.function.DoubleSupplier;

import us.ihmc.commons.Conversions;

/**
 * <p>Minimal stopwatch with a friendly API. Measures durations of time.
 * Features the ability to suspend and resume measurement of the current lap.</p>
 * 
 * <p>Alternative to {@link org.apache.commons.lang3.time.StopWatch StopWatch} in
 * Apache Commons Lang with the following functional differences:</p>
 * 
 * <li>Uses double precision instead of long.</p>
 * <li>Uses seconds as the default unit instead of milliseconds.</li>
 * <li>Provides a more fluent interface with the concept of laps.</li>
 * <li>Keeps track of average lap.</li>
 * <li>API is safe. No exceptions are ever thrown.</li>
 */
public class Stopwatch
{
   private final DoubleSupplier nowSupplier;

   private double lapStart;
   private long lapCount;
   private double recordedLapTotal;

   private boolean suspended;
   private double suspendStart;
   private double resumedSuspensionTotal;

   /**
    * <p>Construct a new Stopwatch.</p>
    * 
    * <p>All methods will return NaN until {@link #start()} or {@link #reset()} is called.</p>
    */
   public Stopwatch()
   {
      nowSupplier = () -> {
         return Conversions.nanosecondsToSeconds(System.nanoTime());
      };
      lapStart = Double.NaN;
   }

   /**
    * Start the clock. Functionally the same as {@link #reset()}.
    * 
    * <p>NOTE: Will reset any suspensions of measurement from previous calls
    * to {@link #suspend()} and {@link #resume()}.</p>
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
    * 
    * <p>NOTE: Will reset any suspensions of measurement from previous calls
    * to {@link #suspend()} and {@link #resume()}.</p>
    */
   public void resetLap()
   {
      lapStart = now();

      resetSuspension();
   }

   /**
    * Reset the stopwatch. Clears average, resets lap, starts the clock.
    * 
    * <p>NOTE: Will reset any suspensions of measurement from previous calls
    * to {@link #suspend()} and {@link #resume()}.</p>
    */
   public void reset()
   {
      lapStart = now();

      resetSuspension();

      lapCount = 0;
      recordedLapTotal = 0.0;
   }

   /**
    * Record a lap. Shortest time since last lap, start, or reset is returned and averaged.
    * 
    * <p>NOTE: Will omit the sum of all suspend durations and
    * automatically resume measurement for the next lap.</p>
    * 
    * @return Lap time.
    */
   public double lap()
   {
      double now = now();
      double lapDuration = lapElapsed(now);
      lapStart = now;

      resetSuspension();

      // for average lap
      lapCount++;
      recordedLapTotal += lapDuration;

      return lapDuration;
   }

   /**
    * Get the average lap duration.
    * 
    * @return Average lap duration.
    */
   public double averageLap()
   {
      return recordedLapTotal / lapCount;
   }

   /**
    * Get the total elapsed time. Since the last reset or start.
    * 
    * @return Total elapsed time.
    */
   public double totalElapsed()
   {
      return recordedLapTotal + lapElapsed(now());
   }

   /**
    * Get the elapsed time in the current lap. Shortest time since last lap, reset, or start.
    * 
    * @return Lap elapsed time.
    */
   public double lapElapsed()
   {
      return lapElapsed(now());
   }

   /**
    * <p>Suspend the measurement of a lap.</p>
    * 
    * <p>Use with {@link #resume()} to omit durations of time from the measurement of a lap.</p>
    * 
    * <p>WARNING: {@link #lap()}, {@link #reset()}, {@link #lapReset()}, and {@link #start()}
    * will reset any suspensions of measurement from previous calls to {@link #suspend()} and
    * {@link #resume()}.</p>
    */
   public void suspend()
   {
      if (!suspended)
      {
         suspended = true;
         suspendStart = now();
      }
   }

   /**
    * <p>Resume the measurement of a lap.</p>
    * 
    * <p>Use with {@link #suspend()} to omit durations of time from the measurement of a lap.</p>
    * 
    * <p>WARNING: {@link #lap()}, {@link #reset()}, {@link #lapReset()}, and {@link #start()}
    * will reset any suspensions of measurement from previous calls to {@link #suspend()} and
    * {@link #resume()}.</p>
    */
   public void resume()
   {
      if (suspended)
      {
         suspended = false;
         resumedSuspensionTotal += now() - suspendStart;
      }
   }

   private double lapElapsed(double now)
   {
      double lapElapsed = now - lapStart;
      lapElapsed -= resumedSuspensionTotal;
      if (suspended)
      {
         lapElapsed -= (now - suspendStart);
      }
      return lapElapsed;
   }

   private double now()
   {
      return nowSupplier.getAsDouble();
   }

   private void resetSuspension()
   {
      suspended = false;
      resumedSuspensionTotal = 0.0;
   }

   /**
    * WARNING: Exclusively for unit testing. Do not use.
    */
   /** package-private */ Stopwatch(DoubleSupplier nowSupplier)
   {
      this.nowSupplier = nowSupplier;
      lapStart = Double.NaN;
   }
}
