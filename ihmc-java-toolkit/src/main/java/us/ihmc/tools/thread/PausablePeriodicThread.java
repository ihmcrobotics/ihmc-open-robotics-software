package us.ihmc.tools.thread;

import java.util.concurrent.ScheduledFuture;

public class PausablePeriodicThread
{
   private final Runnable runnable;
   private final double period;
   private final ExceptionHandlingThreadScheduler scheduler;

   private volatile ScheduledFuture<?> scheduled;

   public PausablePeriodicThread(Runnable runnable, double period, String name)
   {
      this.runnable = runnable;
      this.period = period;

      scheduler = new ExceptionHandlingThreadScheduler(name);
   }

   public void start()
   {
      if (scheduled == null || scheduled.isDone())
      {
         scheduled = scheduler.schedule(runnable, period);
      }
   }

   public void stop()
   {
      if (scheduled != null && !scheduled.isCancelled())
      {
         scheduled.cancel(false);  // does not block
      }
   }
}
