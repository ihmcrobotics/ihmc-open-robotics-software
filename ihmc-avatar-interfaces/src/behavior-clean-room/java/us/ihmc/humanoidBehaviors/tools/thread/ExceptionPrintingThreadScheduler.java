package us.ihmc.humanoidBehaviors.tools.thread;

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

   public ExceptionPrintingThreadScheduler(String name)
   {
      this.executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory(name));
   }

   public ScheduledFuture<?> schedule(Runnable runnable, long period, TimeUnit timeunit)
   {
      this.runnable = runnable;
      if (running)
      {
         throw new RuntimeException("Thread has already been scheduled");
      }

      ScheduledFuture<?> scheduledFuture = executorService.scheduleAtFixedRate(this::printingRunnableWrapper, 0, period, timeunit);
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
         LogTools.error(t.getMessage());
         LogTools.error("{} is terminating due to an exception.", Thread.currentThread().getName());
         throw t;
      }
   }

   public void shutdown()
   {
      executorService.shutdown();
   }

   public void awaitTermination(long timeout, TimeUnit timeUnit) throws InterruptedException
   {
      executorService.awaitTermination(timeout, timeUnit);
   }
}
