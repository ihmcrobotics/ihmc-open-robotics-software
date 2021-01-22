package us.ihmc.tools.thread;

import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.*;

public class MissingThreadTools
{
   public static ThreadFactory createNamedThreadFactory(String prefix, boolean daemon)
   {
      boolean includePoolInName = true;
      boolean includeThreadNumberInName = true;
      return ThreadTools.createNamedThreadFactory(prefix, includePoolInName, includeThreadNumberInName, daemon, Thread.NORM_PRIORITY);
   }

   public static ResettableExceptionHandlingExecutorService newSingleThreadExecutor(String prefix)
   {
      return newSingleThreadExecutor(prefix, false);
   }

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
            createNamedThreadFactory(prefix, daemon),
            new ThreadPoolExecutor.AbortPolicy())
      );
   }
}
