package us.ihmc.tools.time;

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

   private double calculateFrequency()
   {
      Long first = pingTimes.peekFirst();
      int pings = pingTimes.size();

      if (first != null && pings > 1)
      {
         long now = System.nanoTime();
         long elapsedNanos = now - first;
         long elapsedNanosAverage = elapsedNanos / (pings - 1);
         double elapsedSecondsAverage = elapsedNanosAverage / NANOS_IN_A_SECOND;
         return UnitConversions.secondsToHertz(elapsedSecondsAverage);
      }

      return 0.0;
   }

   public void ping() {
      pingTimes.add(System.nanoTime());

      double frequency = calculateFrequency();
      while (frequency > 0.0 && pingTimes.size() > (frequency * 10))
      {
         pingTimes.remove();
      }
   }

   public double getFrequency()
   {
      double frequency = calculateFrequency();

      while (frequency > 0.0 && pingTimes.size() > (frequency * 10))
      {
         pingTimes.remove();
      }

      return frequency;
   }

   public void destroy()
   {
      loggingThreadRunning = false;
   }
}
