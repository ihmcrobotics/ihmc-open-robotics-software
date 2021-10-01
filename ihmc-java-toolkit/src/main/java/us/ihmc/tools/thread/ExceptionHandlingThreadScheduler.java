package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

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
   private final ExceptionHandler exceptionHandler;
   private final long crashesBeforeGivingUp;
   private long crashCount = 0;
   private Runnable runnable;
   private ScheduledFuture<?> scheduledFuture;
   private volatile boolean isRunningTask = false;

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
      this(name, exceptionHandler, 0);
   }

   /**
    * Try to handle the exception but give up after N tries.
    *
    * @param prefix thread name prefix
    * @param exceptionHandler
    * @param crashesBeforeGivingUp
    */
   public ExceptionHandlingThreadScheduler(String prefix, ExceptionHandler exceptionHandler, long crashesBeforeGivingUp)
   {
      this(prefix,exceptionHandler, crashesBeforeGivingUp, false);
   }

   public ExceptionHandlingThreadScheduler(String prefix, ExceptionHandler exceptionHandler, long crashesBeforeGivingUp, boolean runAsDaemon)
   {
      executorService = runAsDaemon ?
            ThreadTools.newSingleDaemonThreadScheduledExecutor(prefix) : ThreadTools.newSingleThreadScheduledExecutor(prefix);
      this.exceptionHandler = exceptionHandler;
      this.crashesBeforeGivingUp = crashesBeforeGivingUp;
   }

   public ScheduledFuture<?> schedule(Runnable runnable, double period)
   {
      return schedule(runnable, Conversions.secondsToNanoseconds(period), TimeUnit.NANOSECONDS);
   }

   public ScheduledFuture<?> schedule(Runnable runnable, long period, TimeUnit timeunit)
   {
      this.runnable = runnable;
      ExceptionTools.handle(() -> scheduledFuture = executorService.scheduleAtFixedRate(this::printingRunnableWrapper, 0, period, timeunit),
                            exception -> LogTools.error(exception.getMessage()));  // reduce error output to just a message

      return scheduledFuture;
   }

   public ScheduledFuture<?> scheduleOnce(Runnable runnable)
   {
      this.runnable = runnable;
      ExceptionTools.handle(() -> scheduledFuture = executorService.schedule(this::printingRunnableWrapper, 0, TimeUnit.MILLISECONDS),
                            exception -> LogTools.error(exception.getMessage()));  // reduce error output to just a message

      return scheduledFuture;
   }

   private void printingRunnableWrapper()
   {
      isRunningTask = true;
      try
      {
         runnable.run();
      }
      catch (Throwable t)
      {
         exceptionHandler.handleException(t);

         ++crashCount;
         if (crashesBeforeGivingUp > 0 // if do not always continue
               && crashCount > crashesBeforeGivingUp) // crash count has now surpassed allowable amount, so give up
         {
            throw t;
         }
      }
      finally
      {
         isRunningTask = false;
      }
   }

   public void shutdown()
   {
      executorService.shutdown();
   }

   public void shutdownNow()
   {
      executorService.shutdownNow();
   }

   public boolean isRunningTask()
   {
      return isRunningTask;
   }
}