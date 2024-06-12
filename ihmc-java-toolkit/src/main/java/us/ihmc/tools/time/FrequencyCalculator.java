package us.ihmc.tools.time;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import javax.annotation.Nullable;
import java.util.UUID;

/**
 * A rolling frequency calculator with an optional logging thread to print the frequency once per second.
 * The frequency is updated on each query to the frequency rather than each new event.
 * Call {@link #ping()} on each new event.
 * Call {@link #getFrequency()} to get the current frequency.
 */
public class FrequencyCalculator
{
   private static final double NANOS_IN_A_SECOND = 1_000_000_000.0;

   private long lastPingTimeNanos;
   private int pingCount;
   private double frequency;
   private long elapsedNanos;

   @Nullable
   private Thread loggingThread;
   private volatile boolean loggingThreadRunning;

   public FrequencyCalculator(boolean enableLoggingThread)
   {
      if (enableLoggingThread)
      {
         String threadID = UUID.randomUUID().toString().substring(0, 5);

         loggingThread = new Thread(() ->
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

   public void ping() {
      long currentTime = System.nanoTime();

      pingCount++;
      elapsedNanos += currentTime - lastPingTimeNanos;
      lastPingTimeNanos = currentTime;
   }

   public double getFrequency()
   {
      if (elapsedNanos >= (NANOS_IN_A_SECOND / 2)) {
         frequency = pingCount / (elapsedNanos / (NANOS_IN_A_SECOND));
         pingCount = 0;
         elapsedNanos = 0;
      }

      return frequency;
   }

   public void destroy()
   {
      loggingThreadRunning = false;
   }
}
