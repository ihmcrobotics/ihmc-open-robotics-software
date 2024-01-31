package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.atomic.AtomicBoolean;

public class RestartableThread
{
   private final String name;
   private final ExceptionHandler exceptionHandler;
   private final boolean runAsDaemon;
   private final RunnableThatThrows runnable;

   private Thread thread;
   private final AtomicBoolean running = new AtomicBoolean(false);

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

   /**
    * Creates a new thread with the parameters used to initialize this class, and starts the new thread
    */
   public void start()
   {
      if (running.compareAndSet(false, true))
      {
         thread = new Thread(() ->
         {
            try
            {
               while (running.get())
               {
                  ExceptionTools.handle(runnable, exceptionHandler);
               }
            }
            finally
            {
               running.set(false);
            }
         }, name);

         thread.setDaemon(runAsDaemon);
         thread.start();
      }
   }

   /**
    * Stops the running thread once it completes the current execution of the runnable
    */
   public void stop()
   {
      running.set(false);
   }

   /**
    * Request the thread to stop and waits until the thread finishes
    */
   public void blockingStop()
   {
      stop();
      if (thread != null)
         ExceptionTools.handle((RunnableThatThrows) thread::join, exceptionHandler);
   }

   public boolean isRunning()
   {
      return running.getPlain();
   }

   boolean isAlive()
   {
      return thread == null ? false : thread.isAlive();
   }
}
