package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
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
            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(exception);
         }
      };
   }
}
