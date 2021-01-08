package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.util.concurrent.*;
import java.util.function.Supplier;

/**
 * Work in progress.
 */
public class ResettableExceptionHandlingExecutorService
{
   public static final ExceptionHandler MESSAGE_AND_TRACE_WITH_THREAD_NAME = exception ->
   {
      if (exception != null)
      {
         LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), exception.getMessage());
         exception.printStackTrace();
      }
   };

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
      execute(runnable, MESSAGE_AND_TRACE_WITH_THREAD_NAME);
   }

   public void execute(Runnable runnable, ExceptionHandler exceptionHandler)
   {
      executorService.execute(runnable, exceptionHandler);
   }

   public Future<Void> submit(Runnable runnable)
   {
      return submit(runnable, MESSAGE_AND_TRACE_WITH_THREAD_NAME);
   }

   public Future<Void> submit(Runnable runnable, ExceptionHandler exceptionHandler)
   {
      return executorService.submit(runnable, exceptionHandler);
   }

   public <V> Future<V> submit(Callable<V> callable, CallableAfterExecuteHandler<V> callableAfterExecuteHandler)
   {
      return executorService.submit(callable, callableAfterExecuteHandler);
   }

   public boolean isExecuting()
   {
      return executorService.getActiveCount() > 0 || executorService.getQueue().size() > 0;
   }

   public void interruptAndReset()
   {
      executorService.interruptRunningAndCancelQueue();
      executorService = executorServiceSupplier.get();
   }

   public void destroy(int maxMillisToWait)
   {
      executorService.shutdownNow();
      if (maxMillisToWait > 0)
         ExceptionTools.handle(() -> executorService.awaitTermination(200, TimeUnit.MILLISECONDS), DefaultExceptionHandler.PRINT_STACKTRACE);
   }
}
