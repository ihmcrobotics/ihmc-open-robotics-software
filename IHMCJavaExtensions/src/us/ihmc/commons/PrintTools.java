package us.ihmc.commons;

/**
 * Conveniently prints useful messages to the console. Uses logging style prefixes
 * and IDE compatible class line number clickable link.
 */
public class PrintTools
{
   /** Debug message prefix */
   public static final String DEBUG = "[DEBUG] ";
   
   /** Info message prefix */
   public static final String INFO = "[INFO] ";
   
   /** Warn message prefix */
   public static final String WARN = "[WARN] ";
   
   /** Error message prefix */
   public static final String ERROR = "[ERROR] ";

   /** For keeping track of the class that actually calls the PrintTools method. */
   private static final int STACK_TRACE_INDEX = 2;
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param debug if false, do nothing
    * @param message message to print
    */
   public static void debug(boolean debug, String message)
   {
      if (debug)
      {
         print(DEBUG, null, message, false);
      }
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param debug if false, do nothing
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void debug(boolean debug, String message, boolean useSystemError)
   {
      if (debug)
      {
         print(DEBUG, null, message, useSystemError);
      }
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    */
   public static void debug(String message)
   {
      print(DEBUG, null, message, false);
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void debug(String message, boolean useSystemError)
   {
      print(DEBUG, null, message, useSystemError);
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param debug if false, do nothing
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    */
   public static void debug(boolean debug, Object containingObjectOrClass, String message)
   {
      if (debug)
      {
         print(DEBUG, containingObjectOrClass, message, false);
      }
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param debug if false, do nothing
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void debug(boolean debug, Object containingObjectOrClass, String message, boolean useSystemError)
   {
      if (debug)
      {
         print(DEBUG, containingObjectOrClass, message, useSystemError);
      }
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    */
   public static void debug(Object containingObjectOrClass, String message)
   {
      print(DEBUG, containingObjectOrClass, message, false);
   }
   
   /**
    * <p>Print a debug level message to the console.</p>
    * 
    * <p>Example: [DEBUG] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void debug(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(DEBUG, containingObjectOrClass, message, useSystemError);
   }

   /**
    * <p>Print a info level message to the console.</p>
    * 
    * <p>Example: [INFO] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    */
   public static void info(Object containingObjectOrClass, String message)
   {
      print(INFO, containingObjectOrClass, message, false);
   }
   
   /**
    * <p>Print a info level message to the console.</p>
    * 
    * <p>Example: [INFO] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    */
   public static void info(String message)
   {
      print(INFO, null, message, false);
   }
   
   /**
    * <p>Print a info level message to the console.</p>
    * 
    * <p>Example: [INFO] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void info(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(INFO, containingObjectOrClass, message, useSystemError);
   }
   
   /**
    * <p>Print a info level message to the console.</p>
    * 
    * <p>Example: [INFO] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void info(String message, boolean useSystemError)
   {
      print(INFO, null, message, useSystemError);
   }

   /**
    * <p>Print a warning level message to the console.</p>
    * 
    * <p>Example: [WARN] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    */
   public static void warn(Object containingObjectOrClass, String message)
   {
      print(WARN, containingObjectOrClass, message, false);
   }
   
   /**
    * <p>Print a warning level message to the console.</p>
    * 
    * <p>Example: [WARN] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    */
   public static void warn(String message)
   {
      print(WARN, null, message, false);
   }
   
   /**
    * <p>Print a warning level message to the console.</p>
    * 
    * <p>Example: [WARN] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void warn(Object containingObjectOrClass, String message, boolean useSystemError)
   {
      print(WARN, containingObjectOrClass, message, useSystemError);
   }
   
   /**
    * <p>Print a warning level message to the console.</p>
    * 
    * <p>Example: [WARN] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    * @param useSystemError if true, use system error, if false, use system out
    */
   public static void warn(String message, boolean useSystemError)
   {
      print(WARN, null, message, useSystemError);
   }

   /**
    * <p>Print a error level message to the console.</p>
    * 
    * <p>Example: [ERROR] (YourClass:19): Your message here</p>
    * 
    * @param containingObjectOrClass passed in calling class to avoid using reflection
    * @param message message to print
    */
   public static void error(Object containingObjectOrClass, String message)
   {
      print(ERROR, containingObjectOrClass, message, true);
   }
   
   /**
    * <p>Print a error level message to the console.</p>
    * 
    * <p>Example: [ERROR] (YourClass:19): Your message here</p>
    * 
    * @param message message to print
    */
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
