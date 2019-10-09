package us.ihmc.humanoidBehaviors.dispatcher;

import us.ihmc.log.LogTools;
import us.ihmc.messager.kryo.MessagerUpdateThread;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

import java.util.concurrent.TimeUnit;

public class BehaviorMessagerUpdater implements MessagerUpdateThread
{
   private final ExceptionHandlingThreadScheduler scheduler;
   private int periodMillis;
   private boolean running = false;

   public BehaviorMessagerUpdater(String name, int periodMillis)
   {
      scheduler = new ExceptionHandlingThreadScheduler(name, t ->
      {
         LogTools.error(t.getMessage());
         t.printStackTrace();
         LogTools.error("{} is crashing due to an exception.", Thread.currentThread().getName());
      }, 5);
      this.periodMillis = periodMillis;
   }

   @Override
   public void start(Runnable runnable)
   {
      if (running)
      {
         throw new RuntimeException("Thread has already been scheduled");
      }

      scheduler.schedule(runnable, periodMillis, TimeUnit.MILLISECONDS);
      running = true;
   }

   @Override
   public void stop()
   {
      scheduler.shutdown();
   }
}
