package us.ihmc.tools.thread;

import java.util.concurrent.ScheduledFuture;

public class PausablePeriodicThread
{
   private final Runnable runnable;
   private final double period;
   private final ExceptionHandlingThreadScheduler scheduler;

   private volatile ScheduledFuture<?> scheduled;

   public PausablePeriodicThread(String name, double period, Runnable runnable)
   {
      this(name, period, 0, runnable);
   }

   public PausablePeriodicThread(String name, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      this.runnable = runnable;
      this.period = period;

      scheduler = new ExceptionHandlingThreadScheduler(name, ExceptionHandlingThreadScheduler.DEFAULT_HANDLER, crashesBeforeGivingUp);
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

   public void setRunning(boolean running)
   {
      if (running)
      {
         start();
      }
      else
      {
         stop();
      }
   }
}
