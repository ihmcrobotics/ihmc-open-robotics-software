package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

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

   }

   public void submit(Runnable runnable, ExceptionHandler exceptionHandler)
   {
      executorService.submit(runnable, future ->
      {
         try
         {
            future.get();
         }
         catch (ExecutionException executionException)
         {
            exceptionHandler.handleException(executionException.getCause());
         }
         catch (CancellationException ce)
         {
            LogTools.warn(1, "Task cancelled");
         }
         catch (InterruptedException ie)
         {
            LogTools.warn(1, "Task interrupted");
            Thread.currentThread().interrupt(); // ignore/reset
         }
      });
   }

   public void destroy()
   {
      executorService.shutdownNow();
      ExceptionTools.handle(() -> executorService.awaitTermination(200, TimeUnit.MILLISECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
   }
}
