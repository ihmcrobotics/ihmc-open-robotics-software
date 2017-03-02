package us.ihmc.commons.exception;

import us.ihmc.tools.io.printing.PrintTools;

public enum DefaultExceptionHandler
{
   PROCEED_SILENTLY, KILL_PROCESS, PRINT_STACKTRACE, PRINT_MESSAGE;
   
   public Object handleException(Throwable throwable)
   {
      switch (this)
      {
      case PROCEED_SILENTLY:
         // do nothing
         break;
      case KILL_PROCESS:
         System.exit(1);
         break;
      case PRINT_STACKTRACE:
         throwable.printStackTrace();
         break;
      case PRINT_MESSAGE:
         PrintTools.error(this, throwable.getMessage());
         break;
      }
      
      return null;
   }
}
