package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.atomic.AtomicBoolean;

public class RestartableThrottledThread
{
   private final String name;
   private final Throttler throttler;
   private final ExceptionHandler exceptionHandler;
   private final boolean runAsDaemon;
   private final RunnableThatThrows runnable;

   private Thread thread;
   private final AtomicBoolean running = new AtomicBoolean(false);

   public RestartableThrottledThread(String name, double runFrequency, RunnableThatThrows runnable)
   {
      this(name, runFrequency, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, false, runnable);
   }

   public RestartableThrottledThread(String name, double runFrequency, ExceptionHandler exceptionHandler, RunnableThatThrows runnable)
   {
      this(name, runFrequency, exceptionHandler, false, runnable);
   }

   public RestartableThrottledThread(String name, double runFrequency, ExceptionHandler exceptionHandler, boolean runAsDaemon, RunnableThatThrows runnable)
   {
      this.name = name;
      this.throttler = new Throttler();
      this.throttler.setFrequency(runFrequency);
      this.exceptionHandler = exceptionHandler;
      this.runAsDaemon = runAsDaemon;
      this.runnable = runnable;
   }

   /**
    * Starts a new thread which runs the passed in Runnable
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
                  throttler.waitAndRun();

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

   // only used for testing
   boolean isAlive()
   {
      return thread == null ? false : thread.isAlive();
   }
}
