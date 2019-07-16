package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * This class first prints any unchecked exception messages and termination message so that it is impossible to miss.
 *
 * It also returns a ScheduledFuture so the exception may be handled, but meant it could no longer implement PeriodicThreadScheduler.
 */
public class ExceptionHandlingThreadScheduler
{
   public static final ExceptionHandler DEFAULT_HANDLER = t ->
   {
      LogTools.error(t.getMessage());
      t.printStackTrace();
      LogTools.error("{} is terminating due to an exception.", Thread.currentThread().getName());
   };

   private volatile ScheduledExecutorService executorService;
   private final String name;
   private final ExceptionHandler exceptionHandler;
   private final Long crashesBeforeGivingUp;
   private long crashCount = 0;
   private boolean running = false;
   private Runnable runnable;
   private ScheduledFuture<?> scheduledFuture;

   /**
    * Normal operation. Just print a good error message and shutdown.
    *
    * @param name thread name
    */
   public ExceptionHandlingThreadScheduler(String name)
   {
      this(name, DEFAULT_HANDLER, 0);
   }

   /** TODO: Add constructor with Exception handler that print only the message N-1 times and print the stack trace when it finally crashes */

   /**
    * Handle the exceptions yourself and recover. Always resume running.
    *
    * @param name thread name
    * @param exceptionHandler
    */
   public ExceptionHandlingThreadScheduler(String name, ExceptionHandler exceptionHandler)
   {
      this(name, exceptionHandler, Long.MIN_VALUE);
   }

   /**
    * Try to handle the exception but give up after N tries.
    *
    * @param name thread name
    * @param exceptionHandler
    * @param crashesBeforeGivingUp
    */
   public ExceptionHandlingThreadScheduler(String name, ExceptionHandler exceptionHandler, long crashesBeforeGivingUp)
   {
      executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
      this.name = name;
      this.exceptionHandler = exceptionHandler;
      this.crashesBeforeGivingUp = crashesBeforeGivingUp;
   }

   public ScheduledFuture<?> schedule(Runnable runnable, double period)
   {
      return schedule(runnable, Conversions.secondsToNanoseconds(period), TimeUnit.NANOSECONDS);
   }

   public ScheduledFuture<?> schedule(Runnable runnable, long period, TimeUnit timeunit)
   {
      if (!running)
      {
         this.runnable = runnable;
         ExceptionTools.handle(() -> scheduledFuture = executorService.scheduleAtFixedRate(this::printingRunnableWrapper, 0, period, timeunit),
                               exception -> LogTools.error(exception.getMessage()));
         running = true;
      }
      else
      {
         LogTools.warn("Thread has already been scheduled");
         new Throwable().printStackTrace();
      }

      return scheduledFuture;
   }

   public ScheduledFuture<?> scheduleOnce(Runnable runnable)
   {
      this.runnable = runnable;
      ExceptionTools.handle(() -> scheduledFuture = executorService.schedule(this::printingRunnableWrapper, 0, TimeUnit.MILLISECONDS),
                            exception -> LogTools.error(exception.getMessage()));
      running = true;

      return scheduledFuture;
   }

   private void printingRunnableWrapper()
   {
      try
      {
         runnable.run();
      }
      catch (Throwable t)
      {
         exceptionHandler.handleException(t);

         ++crashCount;
         if (crashesBeforeGivingUp.longValue() != Long.MIN_VALUE // if do not always continue
               && crashCount > crashesBeforeGivingUp) // crash count has now surpassed allowable amount, so give up
         {
            throw t;
         }
      }
   }

   public void shutdown()
   {
      executorService.shutdown();

      ThreadTools.startAThread(() ->  // start a thread to wait for termination to set running to false to prevent double starting
      {
         ExceptionTools.handle(() -> awaitTermination(10, TimeUnit.SECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
         running = false;
      }, "ShutdownThread");
   }

   public void awaitTermination(long timeout, TimeUnit timeUnit) throws InterruptedException
   {
      executorService.awaitTermination(timeout, timeUnit);
      running = false;

      executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name)); // allow to be reused
   }
}