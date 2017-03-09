package us.ihmc.commons.exception;

import us.ihmc.commons.PrintTools;

/**
 * Create awareness, expliciness, and ease of handling exceptions in default or common ways.
 */
public enum DefaultExceptionHandler
{
   /** Does nothing. */
   PROCEED_SILENTLY,

   /** Runs System.exit(1), killing the process and indicating failure. */
   KILL_PROCESS,

   /** Prints the stack trace. */
   PRINT_STACKTRACE,

   /** Prints the throwable's message in a friendly way using {@link PrintTools} */
   PRINT_MESSAGE;

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
