package us.ihmc.tools.thread;

import us.ihmc.log.LogTools;

import java.util.function.Consumer;

@FunctionalInterface
public interface CallableResultHandler<V>
{
   void handle(V result, Throwable exception, boolean cancelled);

   static <V> CallableResultHandler<V> DEFAULT(Consumer<V> resultHandler)
   {
      return (result, exception, cancelled) ->
      {
         if (cancelled)
         {
            LogTools.warn(1, "Task cancelled");
         }
         else if (exception != null)
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
