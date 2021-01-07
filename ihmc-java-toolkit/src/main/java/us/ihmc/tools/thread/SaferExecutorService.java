package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.*;

/**
 * Work in progress.
 */
public class SaferExecutorService
{
   private final ExceptionHandlingThreadPoolExecutor executorService;

   public SaferExecutorService(ExceptionHandlingThreadPoolExecutor executorService)
   {
      this.executorService = executorService;
   }

   public void execute(Runnable runnable)
   {
      execute(runnable, ExecutionResultHandler.DEFAULT());
   }

   public void execute(Runnable runnable, ExecutionResultHandler executionResultHandler)
   {
      executorService.execute(runnable, executionResultHandler);
   }

   public void submit(Runnable runnable)
   {
      submit(runnable, ExecutionResultHandler.DEFAULT());
   }

   public void submit(Runnable runnable, ExecutionResultHandler executionResultHandler)
   {
      executorService.submit(runnable, executionResultHandler);
   }

   public void destroy()
   {
      executorService.shutdownNow();
      ExceptionTools.handle(() -> executorService.awaitTermination(200, TimeUnit.MILLISECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
   }
}
