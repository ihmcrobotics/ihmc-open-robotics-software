package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.*;
import java.util.concurrent.locks.LockSupport;

public class MissingThreadTools
{
   /**
    * Guarantees a sleep of a minimum duration in floating point seconds
    * using {@link LockSupport#parkNanos}. It will always sleep a little too long.
    * The amount overslept probably varies by system, but it has been observed to
    * be less than half a millisecond.
    *
    * {@link ThreadTools#sleepSeconds} can actually return early because it
    * cuts off the subnanosecond part, allowing it to undersleep by a nanosecond
    * at most.
    *
    * @param duration to sleep in seconds
    * @return Exactly how long it actually slept in seconds
    */
   public static double sleepAtLeast(double duration)
   {
      double startTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      double amountSlept = 0.0;
      do
      {
         double nextDuration = duration - amountSlept;

         sleep(nextDuration);

         amountSlept = Conversions.nanosecondsToSeconds(System.nanoTime()) - startTime;
      }
      while (amountSlept < duration);
      return amountSlept;
   }

   /** Uses {@link LockSupport#parkNanos} to sleep for floating point seconds. */
   public static void sleep(double seconds)
   {
      double floatingNanos = seconds * 1e9;
      long nanoseconds = (long) floatingNanos;

      if (floatingNanos > nanoseconds) // Take nanosecond ceiling instead of floor
         ++nanoseconds;

      LockSupport.parkNanos(nanoseconds); // More accurate than Thread.sleep
   }

   public static ThreadFactory createNamedThreadFactory(String prefix, boolean daemon)
   {
      boolean includePoolInName = true;
      boolean includeThreadNumberInName = true;
      return ThreadTools.createNamedThreadFactory(prefix, includePoolInName, includeThreadNumberInName, daemon, Thread.NORM_PRIORITY);
   }

   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix)
   {
      return newSingleThreadExecutor(prefix, false);
   }

   /**
    * Creates a single thread executor with no limit on the queue size.
    */
   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix, boolean daemon)
   {
      return newSingleThreadExecutor(prefix, daemon, -1);
   }

   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix, boolean daemon, int queueSize)
   {
      int corePoolSize = 1;
      int maximumPoolSize = 1;
      long keepAliveTime = 0L;
      return new ResettableExceptionHandlingExecutorService(() -> new ExceptionHandlingThreadPoolExecutor(
            corePoolSize,
            maximumPoolSize,
            keepAliveTime,
            TimeUnit.MILLISECONDS,
            queueSize < 0 ? new LinkedBlockingQueue<>() : new ArrayBlockingQueue<>(queueSize),
            createNamedThreadFactory(prefix, daemon),
            new ThreadPoolExecutor.AbortPolicy())
      );
   }

   public static Thread startAsDaemon(String threadName, ExceptionHandler exceptionHandler, RunnableThatThrows runnable)
   {
      return ThreadTools.startAsDaemon(() -> ExceptionTools.handle(runnable, exceptionHandler), threadName);

   }
   public static Thread startAsDaemon(String threadName, double period, RunnableThatThrows runnable)
   {
      Throttler throttler = new Throttler();
      return startAsDaemon(threadName, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
      {
         while (true)
         {
            throttler.waitAndRun(period);
            runnable.run();
         }
      });
   }

   public static Thread startAThread(String threadName, ExceptionHandler exceptionHandler, RunnableThatThrows runnable)
   {
      return ThreadTools.startAThread(() -> ExceptionTools.handle(runnable, exceptionHandler), threadName);
   }
}
