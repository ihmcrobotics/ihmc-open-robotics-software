package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;

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
   }

   public void start()
   {
      running = true;
      thread = new Thread(() ->
      {
         try
         {
            runnable.run();
         }
         catch (Throwable throwable)
         {
            // Interrupt used to stop thread. Don't handle interrupted exceptions
            if (!(throwable instanceof InterruptedException))
               exceptionHandler.handleException(throwable);

            running = false;
         }
      }, name);

      thread.setDaemon(runAsDaemon);
      thread.start();
   }

   public void stop()
   {
      thread.interrupt();
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
