package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.*;

public class MissingThreadTools
{
   /**
    * {@link ThreadTools#sleepSeconds} can actually return early.
    * Here we guarantee to sleep the entire duration, which means it will
    * always sleep a little too long. The amount probably varies by system,
    * but it has been observed to be less than half a millisecond.
    *
    * @param duration to sleep in seconds
    */
   public static double sleepAtLeast(double duration)
   {
      double amountSlept = 0.0;
      while (amountSlept < duration)
      {
         amountSlept += Conversions.nanosecondsToSeconds(ThreadTools.sleepSeconds(duration - amountSlept));
      }
      return amountSlept;
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
