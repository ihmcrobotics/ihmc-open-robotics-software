package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * TODO: Add shutdown exception limit?
 */
public class SingleThreadSizeOneQueueExecutor
{
   private final String prefix;
   private ThreadPoolExecutor executor;
   private ArrayBlockingQueue<Runnable> sizeOneQueue;

   public SingleThreadSizeOneQueueExecutor(String prefix)
   {
      this.prefix = prefix;

      recreate();
   }
   private void recreate()
   {
      sizeOneQueue = new ArrayBlockingQueue<>(1);
      executor = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS, sizeOneQueue, ThreadTools.createNamedThreadFactory(prefix));
   }

   public void queueExecution(Runnable runnable)
   {
      try
      {
         sizeOneQueue.remove();
      }
      catch (NoSuchElementException e)
      {
         // This is fine. We are just trying to replace the one element.
      }

      executor.execute(runnable);
   }

   public void interruptAndReset()
   {
      executor.shutdownNow();
      recreate();
   }

   // TODO: Why isn't this used?
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
