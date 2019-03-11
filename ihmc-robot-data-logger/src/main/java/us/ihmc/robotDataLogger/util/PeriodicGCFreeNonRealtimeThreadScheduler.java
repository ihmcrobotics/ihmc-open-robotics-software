package us.ihmc.robotDataLogger.util;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.LockSupport;

import us.ihmc.util.PeriodicThreadScheduler;

public class PeriodicGCFreeNonRealtimeThreadScheduler implements PeriodicThreadScheduler
{
   private class FixedRateThread extends Thread
   {
      private final long dtInNanos;

      public FixedRateThread(long dtInNanos)
      {
         super(name + "-thread");
         this.dtInNanos = dtInNanos;
      }

      @Override
      public void run()
      {
         long nextWakeTime = System.nanoTime() + dtInNanos;
         while (running)
         {
            runnable.run();

            nextWakeTime += dtInNanos;
            long currentTime = System.nanoTime();
            if (currentTime < nextWakeTime)
            {
               LockSupport.parkNanos(nextWakeTime - currentTime);
            }
            else
            {
               nextWakeTime = currentTime;
            }

         }
      }

   }

   private final String name;
   private volatile boolean running = false;
   private FixedRateThread thread;
   private Runnable runnable;

   public PeriodicGCFreeNonRealtimeThreadScheduler(String name)
   {
      this.name = name;
   }

   @Override
   public synchronized void schedule(Runnable runnable, long period, TimeUnit timeunit)
   {
      if (this.running)
      {
         throw new RuntimeException("Thread has already been scheduled");
      }

      this.runnable = runnable;
      this.running = true;
      this.thread = new FixedRateThread(timeunit.toNanos(period));
      this.thread.start();
   }

   @Override
   public synchronized void shutdown()
   {
      running = false;
   }

   @Override
   public void awaitTermination(long timeout, TimeUnit timeUnit) throws InterruptedException
   {
      thread.join(TimeUnit.MILLISECONDS.convert(timeout, timeUnit));
   }

}