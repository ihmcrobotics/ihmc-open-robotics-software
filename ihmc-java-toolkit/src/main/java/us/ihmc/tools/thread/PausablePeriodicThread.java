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
      this(name, period, false, runnable);
   }

   public PausablePeriodicThread(String name, double period, int crashesBeforeGivingUp, Runnable runnable)
   {
      this(name, period, crashesBeforeGivingUp, false, runnable);
   }

   public PausablePeriodicThread(String name, double period, boolean runAsDaemon, Runnable runnable)
   {
      this(name, period, 0, runAsDaemon, runnable);
   }

   public PausablePeriodicThread(String name, double period, int crashesBeforeGivingUp, boolean runAsDaemon, Runnable runnable)
   {
      this.runnable = runnable;
      this.period = period;

      scheduler = new ExceptionHandlingThreadScheduler(name, ExceptionHandlingThreadScheduler.DEFAULT_HANDLER, crashesBeforeGivingUp, runAsDaemon);
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

   public void destroy()
   {
      stop();
      scheduler.shutdown();
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

   public boolean isRunning()
   {
      if (scheduled == null)
      {
         return false; // never was running
      }
      else if (scheduler.isRunningTask())
      {
         return true; // it's in the middle of computing
      }
      else
      {
         // if it's not done, then we're still going; nobody called stop yet
         // if it is done, then someone called stop and it's finished it's running task because
         // we got past the above condition
         return !scheduled.isDone();
      }
   }
}
