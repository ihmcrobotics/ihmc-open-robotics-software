package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.*;
import java.util.function.Supplier;

/**
 * Work in progress.
 */
public class ResettableExceptionHandlingExecutorService
{
   private final Supplier<ExceptionHandlingThreadPoolExecutor> executorServiceSupplier;
   private ExceptionHandlingThreadPoolExecutor executorService;

   public ResettableExceptionHandlingExecutorService(Supplier<ExceptionHandlingThreadPoolExecutor> executorServiceSupplier)
   {
      this.executorServiceSupplier = executorServiceSupplier;

      executorService = executorServiceSupplier.get();
   }

   public void clearTaskQueue()
   {
      executorService.getQueue().clear();
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

   public boolean isExecuting()
   {
      return executorService.getActiveCount() > 0 || executorService.getQueue().size() > 0;
   }

   public void interruptAndReset()
   {
      executorService.shutdownNow();
      executorService = executorServiceSupplier.get();
   }

   public void destroy()
   {
      executorService.shutdownNow();
      ExceptionTools.handle(() -> executorService.awaitTermination(200, TimeUnit.MILLISECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
   }
}
