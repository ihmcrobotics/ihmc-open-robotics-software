package us.ihmc.tools.time;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;

import java.util.Deque;
import java.util.LinkedList;
import java.util.UUID;

/**
 * A rolling average frequency calculator with an optional logging thread to print the frequency once per second.
 * The frequency is calculated on each query to the frequency rather than each new event.
 * Call {@link #ping()} on each new event.
 * Call {@link #getFrequency()} to get the current frequency.
 */
public class FrequencyCalculator
{
   private static final double NANOS_IN_A_SECOND = 1_000_000_000.0;

   private double frequency;
   private final Deque<Long> pingTimes = new LinkedList<>();

   private volatile boolean loggingThreadRunning;

   public FrequencyCalculator(boolean enableLoggingThread)
   {
      if (enableLoggingThread)
      {
         String threadID = UUID.randomUUID().toString().substring(0, 5);

         Thread loggingThread = new Thread(() ->
         {
            loggingThreadRunning = true;

            while (loggingThreadRunning)
            {
               LogTools.info("FrequencyCalculator[" + threadID + "] average rate: " + getFrequency());

               ThreadTools.sleep(1000);
            }
         }, getClass().getSimpleName() + "-" + threadID);

         loggingThread.start();
      }
   }

   public FrequencyCalculator()
   {
      this(false);
   }

   private double calculateFrequency(boolean decay)
   {
      Long first = pingTimes.peekFirst();
      Long last = decay ? Long.valueOf(System.nanoTime()) : pingTimes.peekLast();
      int pings = pingTimes.size();

      if (first != null && last != null && pings > 1)
      {
         long elapsedNanos = last - first;
         double elapsedSeconds = Conversions.nanosecondsToSeconds(elapsedNanos);
         double elapsedSecondsAverage = elapsedSeconds / (pings - 1);
         return UnitConversions.secondsToHertz(elapsedSecondsAverage);
      }

      return 0.0;
   }

   public void ping() {
      pingTimes.add(System.nanoTime());

      double frequency = calculateFrequency(false);
      while (frequency > 0.0 && pingTimes.size() > (frequency * 10))
      {
         pingTimes.removeFirst();
      }

      this.frequency = frequency;
   }

   public double getFrequency()
   {
      return frequency;
   }

   public double getFrequencyDecaying()
   {
      return calculateFrequency(true);
   }

   public void destroy()
   {
      loggingThreadRunning = false;
   }
}
