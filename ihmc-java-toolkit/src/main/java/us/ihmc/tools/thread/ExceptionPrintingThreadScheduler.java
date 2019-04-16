package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
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
public class ExceptionPrintingThreadScheduler
{
   private final ScheduledExecutorService executorService;
   private boolean running = false;
   private Runnable runnable;
   private ScheduledFuture<?> scheduledFuture;

   public ExceptionPrintingThreadScheduler(String name)
   {
      this.executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   }

   public ScheduledFuture<?> schedule(Runnable runnable, long period, TimeUnit timeunit)
   {
      if (!running)
      {
         this.runnable = runnable;
         scheduledFuture = executorService.scheduleAtFixedRate(this::printingRunnableWrapper, 0, period, timeunit);
         running = true;
      }
      else
      {
         LogTools.warn("Thread has already been scheduled");
      }

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
         LogTools.error(t.getMessage());
         t.printStackTrace();
         LogTools.error("{} is terminating due to an exception.", Thread.currentThread().getName());
         throw t;
      }
   }

   public void shutdown()
   {
      executorService.shutdown();

      new Thread(() ->  // start a thread to wait for termination to set running to false to prevent double starting
                 {
                    ExceptionTools.handle(() -> awaitTermination(10, TimeUnit.SECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
                    running = false;
                 }).start();
   }

   public void awaitTermination(long timeout, TimeUnit timeUnit) throws InterruptedException
   {
      executorService.awaitTermination(timeout, timeUnit);
      running = false;
   }
}
