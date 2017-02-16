package us.ihmc.tools.io.printing;

public class PrintTools
{
   public static final String DEBUG = "[DEBUG] ";
   public static final String INFO = "[INFO] ";
   public static final String WARN = "[WARN] ";
   public static final String ERROR = "[ERROR] ";

   private static final int STACK_TRACE_INDEX = 2;
   
   public static void debug(boolean debug, String message)
   {
      if (debug)
      {
         print(DEBUG, null, message, false);
      }
   }
   
   public static void debug(boolean debug, String message, boolean useSystemError)
   {
      if (debug)
      {
         print(DEBUG, null, message, useSystemError);
      }
   }
   
   public static void debug(String message)
   {
      print(DEBUG, null, message, false);
   }
   
   public static void debug(String message, boolean useSystemError)
   {
      print(DEBUG, null, message, useSystemError);
   }
   
   public static void debug(boolean debug, Object containingObjectOrClass, String message)
   {
      if (debug)
      {
         print(DEBUG, containingObjectOrClass, message, false);
      }
   }
   
   public static void debug(boolean debug, Object containingObjectOrClass, String message, boolean useSystemError)
   {
      if (debug)
      {
         print(DEBUG, containingObjectOrClass, message, useSystemError);
      }
   }
   
   public static void debug(Object containingObjectOrClass, String message)
   {
      print(DEBUG, containingObjectOrClass, message, false);
   }
   
   public static void debug(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(DEBUG, containingObjectOrClass, message, useSystemError);
   }

   public static void info(Object containingObjectOrClass, String message)
   {
      print(INFO, containingObjectOrClass, message, false);
   }
   
   public static void info(String message)
   {
      print(INFO, null, message, false);
   }
   
   public static void info(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(INFO, containingObjectOrClass, message, useSystemError);
   }
   
   public static void info(String message, boolean useSystemError)
   {
      print(INFO, null, message, useSystemError);
   }

   public static void warn(Object containingObjectOrClass, String message)
   {
      print(WARN, containingObjectOrClass, message, false);
   }
   
   public static void warn(String message)
   {
      print(WARN, null, message, false);
   }
   
   public static void warn(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(WARN, containingObjectOrClass, message, useSystemError);
   }
   
   public static void warn(String message, boolean useSystemError)
   {
      print(WARN, null, message, useSystemError);
   }

   public static void error(Object containingObjectOrClass, String message)
   {
      print(ERROR, containingObjectOrClass, message, true);
   }
   
   public static void error(String message)
   {
      print(ERROR, null, message, true);
   }
   
   private static void print(String level, Object containingObjectOrClass, String message, boolean useSystemError)
   {
      Throwable throwable = new Throwable();
      
      int lineNumber = -1;
      String className;
      if (containingObjectOrClass == null)
      {
         String[] classNameSplit = throwable.getStackTrace()[STACK_TRACE_INDEX].getClassName().split("\\.");
         className = classNameSplit[classNameSplit.length - 1].split("\\$")[0];
         lineNumber = throwable.getStackTrace()[STACK_TRACE_INDEX].getLineNumber();
      }
      else
      {
         className = containingObjectOrClass.getClass().getSimpleName();

         for (StackTraceElement stackTraceElement : throwable.getStackTrace())
         {
            if (stackTraceElement.getClassName().endsWith(className))
            {
               lineNumber = stackTraceElement.getLineNumber();
               break;
            }
         }

         if (lineNumber == -1)
            lineNumber = throwable.getStackTrace()[STACK_TRACE_INDEX].getLineNumber();
         
         if (containingObjectOrClass instanceof Class<?>)
            className = ((Class<?>) containingObjectOrClass).getSimpleName();
      }
      
      String clickableLocation = "(" + className + ".java:" + lineNumber + ")";

      String finalOutput = level + clickableLocation + ": " + message;

      if (useSystemError)
         System.err.println(finalOutput);
      else
         System.out.println(finalOutput);
   }
}
