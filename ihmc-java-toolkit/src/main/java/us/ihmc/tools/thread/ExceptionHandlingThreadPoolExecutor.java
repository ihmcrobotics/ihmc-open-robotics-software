package us.ihmc.tools.thread;

import us.ihmc.commons.exception.ExceptionHandler;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.*;

public class ExceptionHandlingThreadPoolExecutor extends ThreadPoolExecutor
{
   private final HashMap<Runnable, ExceptionHandler> afterExecuteHandlers = new HashMap<>();

   public ExceptionHandlingThreadPoolExecutor(int corePoolSize,
                                              int maximumPoolSize,
                                              long keepAliveTime,
                                              TimeUnit unit,
                                              BlockingQueue<Runnable> workQueue,
                                              ThreadFactory threadFactory,
                                              RejectedExecutionHandler handler)
   {
      super(corePoolSize, maximumPoolSize, keepAliveTime, unit, workQueue, threadFactory, handler);
   }

   public void execute(Runnable runnable, ExceptionHandler exceptionHandler)
   {
      afterExecuteHandlers.put(runnable, exceptionHandler);
      execute(runnable);
   }

   public Future<Void> submit(Runnable task, ExceptionHandler exceptionHandler)
   {
      if (task == null) throw new NullPointerException();
      RunnableFuture<Void> futureTask = newTaskFor(task, null);
      afterExecuteHandlers.put(futureTask, throwable ->
      {
         try
         {
            futureTask.get();
         }
         catch (ExecutionException executionException)
         {
            exceptionHandler.handleException(executionException.getCause());
         }
         catch (InterruptedException interruptedException) // if the get() above gets interrupted; expected to never happen
         {
            exceptionHandler.handleException(interruptedException);
         }
         catch (CancellationException cancellationException)
         {
            throw new RuntimeException("This should not be possible. If the future was cancelled it wouldn't get to afterExecute()");
         }
      });
      execute(futureTask);
      return futureTask;
   }

   public <V> Future<V> submit(Callable<V> task, CallableAfterExecuteHandler<V> callableAfterExecuteHandler)
   {
      if (task == null) throw new NullPointerException();
      RunnableFuture<V> futureTask = newTaskFor(task);
      afterExecuteHandlers.put(futureTask, throwable ->
      {
         try
         {
            callableAfterExecuteHandler.handle(futureTask.get(), null);
         }
         catch (ExecutionException executionException)
         {
            callableAfterExecuteHandler.handle(null, executionException.getCause());
         }
         catch (InterruptedException interruptedException) // if the get() above gets interrupted; expected to never happen
         {
            callableAfterExecuteHandler.handle(null, interruptedException);
         }
         catch (CancellationException cancellationException)
         {
            throw new RuntimeException("This should not be possible. If the future was cancelled it wouldn't get to afterExecute()");
         }
      });
      execute(futureTask);
      return futureTask;
   }

   public void interruptRunningAndCancelQueue()
   {
      List<Runnable> queuedTasks = shutdownNow();
      for (Runnable queuedTask : queuedTasks)
      {
         if (queuedTask instanceof Future<?>)
         {
            ((Future<?>) queuedTask).cancel(false);
         }
      }
   }

   @Override
   protected void afterExecute(Runnable runnableFuture, Throwable throwable)
   {
      super.afterExecute(runnableFuture, throwable); // fluff pretty much, super has no implementation

      afterExecuteHandlers.remove(runnableFuture).handleException(throwable);
   }
}
