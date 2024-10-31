package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.LockSupport;

public class MissingThreadTools
{
   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix)
   {
      return newSingleThreadExecutor(prefix, false);
   }

   /**
    * Creates a single thread executor with no limit on the queue size.
    */
   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix, boolean daemon)
   {
      return newSingleThreadExecutor(prefix, daemon, -1);
   }

   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix, boolean daemon, int queueSize)
   {
      int corePoolSize = 1;
      int maximumPoolSize = 1;
      long keepAliveTime = 0L;
      return new ResettableExceptionHandlingExecutorService(() -> new ExceptionHandlingThreadPoolExecutor(
            corePoolSize,
            maximumPoolSize,
            keepAliveTime,
            TimeUnit.MILLISECONDS,
            queueSize < 0 ? new LinkedBlockingQueue<>() : new ArrayBlockingQueue<>(queueSize),
            ThreadTools.createNamedThreadFactory(prefix, daemon),
            new ThreadPoolExecutor.AbortPolicy())
      );
   }
}
