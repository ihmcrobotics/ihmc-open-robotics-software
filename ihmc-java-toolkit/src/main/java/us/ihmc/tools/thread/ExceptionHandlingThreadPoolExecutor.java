package us.ihmc.tools.thread;

import us.ihmc.commons.exception.ExceptionHandler;

import java.util.HashMap;
import java.util.concurrent.*;
import java.util.function.Consumer;

public class ExceptionHandlingThreadPoolExecutor extends ThreadPoolExecutor
{
   private final HashMap<Runnable, Consumer<Future<?>>> afterSubmitHandlers = new HashMap<>();
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

   public Future<?> submit(Runnable task, Consumer<Future<?>> afterExecute)
   {
      if (task == null) throw new NullPointerException();
      RunnableFuture<Void> futureTask = newTaskFor(task, null);
      afterSubmitHandlers.put(futureTask, afterExecute);
      execute(futureTask);
      return futureTask;
   }

   @Override
   protected void afterExecute(Runnable runnableFuture, Throwable throwable)
   {
      super.afterExecute(runnableFuture, throwable); // fluff pretty much, super has no implementation

      if (throwable == null && runnableFuture instanceof Future<?>) // used submit
      {
         Future<?> castedFuture = (Future<?>) runnableFuture;
         afterSubmitHandlers.computeIfPresent(runnableFuture, (future, listener) ->
         {
            listener.accept(castedFuture);
            return null;
         });
      }
      else if (throwable != null) // used execute directly
      {
         afterExecuteHandlers.computeIfPresent(runnableFuture, (runnable, handler) ->
         {
            handler.handleException(throwable);
            return null;
         });
      }
   }
}
