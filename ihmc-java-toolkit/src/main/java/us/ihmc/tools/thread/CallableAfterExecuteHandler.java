package us.ihmc.tools.thread;

import java.util.function.Consumer;

@FunctionalInterface
public interface CallableAfterExecuteHandler<V>
{
   void handle(V result, Throwable exception);

   static <V> CallableAfterExecuteHandler<V> DEFAULT(Consumer<V> resultHandler)
   {
      return (result, exception) ->
      {
         if (exception != null)
         {
            ResettableExceptionHandlingExecutorService.MESSAGE_AND_TRACE_WITH_THREAD_NAME.handleException(exception);
         }
         else
         {
            resultHandler.accept(result);
         }
      };
   }
}
