package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executor;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class SingleThreadSizeOneQueueExecutor
{
   private final Executor executor;

   public SingleThreadSizeOneQueueExecutor(String prefix)
   {
      ArrayBlockingQueue<Runnable> sizeOneQueue = new ArrayBlockingQueue<>(1);
      executor = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS, sizeOneQueue, ThreadTools.createNamedThreadFactory(prefix));
   }

   public void execute(Runnable runnable)
   {
      executor.execute(() -> exceptionHandlingWrapper(runnable));
   }

   private void exceptionHandlingWrapper(Runnable runnable)
   {
      try
      {
         runnable.run();
      }
      catch (Throwable t)
      {
         LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), t.getMessage());
         t.printStackTrace();
         throw t;
      }
   }
}
