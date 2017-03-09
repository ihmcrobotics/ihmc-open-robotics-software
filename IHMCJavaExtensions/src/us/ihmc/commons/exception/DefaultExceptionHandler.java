package us.ihmc.commons.exception;

import us.ihmc.commons.PrintTools;

/**
 * Create awareness, expliciness, and ease of handling exceptions in default or common ways.
 */
public enum DefaultExceptionHandler
{
   PROCEED_SILENTLY, KILL_PROCESS, PRINT_STACKTRACE, PRINT_MESSAGE;
   
   /**
    * Handles the throwable in one of the default ways.
    * 
    * @param throwable to be handled
    * @return Null casted to Object for convenience.
    */
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
