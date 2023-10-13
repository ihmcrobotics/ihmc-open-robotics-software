package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

public class RestartableThread
{
   private final String name;
   private final ExceptionHandler exceptionHandler;
   private final boolean runAsDaemon;
   private final RunnableThatThrows runnable;

   private Thread thread;
   private boolean running = false;

   public RestartableThread(String name, RunnableThatThrows runnable)
   {
      this(name, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, false, runnable);
   }

   public RestartableThread(String name, boolean runAsDaemon, RunnableThatThrows runnable)
   {
      this(name, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, runAsDaemon, runnable);
   }

   public RestartableThread(String name, ExceptionHandler exceptionHandler, RunnableThatThrows runnable)
   {
      this(name, exceptionHandler, false, runnable);
   }

   public RestartableThread(String name, ExceptionHandler exceptionHandler, boolean runAsDaemon, RunnableThatThrows runnable)
   {
      this.name = name;
      this.exceptionHandler = exceptionHandler;
      this.runAsDaemon = runAsDaemon;
      this.runnable = runnable;
      this.thread = new Thread(); // initialize thread so stopping without starting doesn't throw exception
   }

   /**
    * Creates a new thread with the parameters used to initialize this class, and starts the new thread
    */
   public void start()
   {
      running = true;
      thread = new Thread(() ->
      {
         try
         {
            while (running)
            {
               ExceptionTools.handle(runnable, exceptionHandler);
            }
         }
         finally
         {
            running = false;
         }
      }, name);

      thread.setDaemon(runAsDaemon);
      thread.start();
   }

   /**
    * Stops the running thread once it completes the current execution of the runnable
    */
   public void stop()
   {
      running = false;
   }

   public boolean isRunning()
   {
      return running;
   }

   boolean isAlive()
   {
      return thread.isAlive();
   }
}
