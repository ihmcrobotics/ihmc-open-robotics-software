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

   public static SaferExecutorService newSingleThreadExecutor(String prefix)
   {
      return newSingleThreadExecutor(prefix, false);
   }

   public static SaferExecutorService newSingleThreadExecutor(String prefix, boolean daemon)
   {
      int corePoolSize = 1;
      int maximumPoolSize = 1;
      long keepAliveTime = 0L;
      ExceptionHandlingThreadPoolExecutor executorService = new ExceptionHandlingThreadPoolExecutor(corePoolSize,
                                                                                                    maximumPoolSize,
                                                                                                    keepAliveTime,
                                                                                                    TimeUnit.MILLISECONDS,
                                                                                                    new LinkedBlockingQueue<>(),
                                                                                                    createNamedThreadFactory(prefix, daemon),
                                                                                                    new ThreadPoolExecutor.AbortPolicy());
      return new SaferExecutorService(executorService);
   }
}
