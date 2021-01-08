package us.ihmc.tools.thread;

import us.ihmc.log.LogTools;

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
            LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), exception.getMessage());
            exception.printStackTrace();
         }
         else
         {
            resultHandler.accept(result);
         }
      };
   }
}
