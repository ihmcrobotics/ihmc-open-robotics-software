package us.ihmc.tools.thread;

import us.ihmc.log.LogTools;

@FunctionalInterface
public interface RunnableResultHandler
{
   void handle(Throwable exception, boolean cancelled);

   static RunnableResultHandler DEFAULT()
   {
      return (exception, cancelled) ->
      {
         if (cancelled)
         {
            LogTools.warn(1, "Task cancelled");
         }
         else
         {
            LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), exception.getMessage());
            exception.printStackTrace();
         }
      };
   }
}
