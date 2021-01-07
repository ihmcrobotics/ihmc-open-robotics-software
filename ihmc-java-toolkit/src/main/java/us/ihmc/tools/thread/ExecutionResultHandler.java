package us.ihmc.tools.thread;

import us.ihmc.log.LogTools;

@FunctionalInterface
public interface ExecutionResultHandler
{
   void handle(Throwable exception, boolean cancelled, boolean interrupted);

   static ExecutionResultHandler DEFAULT()
   {
      return (exception, cancelled, interrupted) ->
      {
         if (cancelled)
         {
            LogTools.warn(1, "Task cancelled");
         }
         else if (interrupted)
         {
            LogTools.warn(1, "Task interrupted");
            Thread.currentThread().interrupt(); // ignore/reset
         }
         else
         {
            LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), exception.getMessage());
            exception.printStackTrace();
         }
      };
   }
}
