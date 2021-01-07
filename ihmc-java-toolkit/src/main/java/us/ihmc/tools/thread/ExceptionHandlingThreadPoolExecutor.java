package us.ihmc.tools.thread;

import java.util.HashMap;
import java.util.concurrent.*;

public class ExceptionHandlingThreadPoolExecutor extends ThreadPoolExecutor
{
   private final HashMap<Runnable, ExecutionResultHandler> executionResultHandlers = new HashMap<>();

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

   public void execute(Runnable runnable, ExecutionResultHandler executionResultHandler)
   {
      executionResultHandlers.put(runnable, executionResultHandler);
      execute(runnable);
   }

   public Future<?> submit(Runnable task, ExecutionResultHandler executionResultHandler)
   {
      if (task == null) throw new NullPointerException();
      RunnableFuture<Void> futureTask = newTaskFor(task, null);
      executionResultHandlers.put(futureTask, executionResultHandler);
      execute(futureTask);
      return futureTask;
   }

//   public <V> Future<V> submit(Callable<V> task, ExecutionResultHandler executionResultHandler)
//   {
//      if (task == null) throw new NullPointerException();
//      RunnableFuture<V> futureTask = newTaskFor(task);
//      executionResultHandlers.put(futureTask, executionResultHandler);
//      execute(futureTask);
//      return futureTask;
//   }

   @Override
   protected void afterExecute(Runnable runnableFuture, Throwable throwable)
   {
      super.afterExecute(runnableFuture, throwable); // fluff pretty much, super has no implementation

      if (throwable == null && runnableFuture instanceof Future<?>) // used submit
      {
         Future<?> castedFuture = (Future<?>) runnableFuture;
         executionResultHandlers.computeIfPresent(runnableFuture, (future, executionResultHandler) ->
         {
            try
            {
               castedFuture.get();
            }
            catch (CancellationException cancellationException)
            {
               executionResultHandler.handle(cancellationException, true, false);
            }
            catch (InterruptedException interruptedException)
            {
               executionResultHandler.handle(interruptedException, false, true);
            }
            catch (ExecutionException executionException)
            {
               executionResultHandler.handle(executionException.getCause(), false, false);
            }
            return null;
         });
      }
      else if (throwable != null) // used execute directly
      {
         executionResultHandlers.computeIfPresent(runnableFuture, (runnable, executionResultHandler) ->
         {
            if (throwable instanceof CancellationException)
            {
               executionResultHandler.handle(throwable, true, false);
            }
            else if (throwable instanceof InterruptedException)
            {
               executionResultHandler.handle(throwable, false, true);
            }
            else // ExecutionException
            {
               executionResultHandler.handle(throwable, false, false);
            }
            return null;
         });
      }
   }
}
