package us.ihmc.tools;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.util.NoSuchElementException;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * The functionality is that the currently executing task (if any) is left to finish as normal.
 * Upon the running task completing, the queued task is then started. The main idea is to prevent
 * running out of date tasks. When a task is queued, if one it already running AND one is queued,
 * the new task will replace the one in the queue. This way, queuing new tasks can happen at high
 * frequency even if the tasks take a long time to run.
 *
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
      executor = new ThreadPoolExecutor(1, 1, 0L, TimeUnit.MILLISECONDS, sizeOneQueue, ThreadTools.createNamedDaemonThreadFactory(prefix));
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

      executor.execute(() -> exceptionHandlingWrapper(runnable));
   }

   public void interruptAndReset()
   {
      executor.shutdownNow();
      recreate();
   }

   public boolean isExecuting()
   {
      return executor.getActiveCount() > 0 || sizeOneQueue.size() > 0;
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
