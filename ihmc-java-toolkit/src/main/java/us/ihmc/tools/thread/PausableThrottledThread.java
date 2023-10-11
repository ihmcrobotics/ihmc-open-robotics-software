package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

public class PausableThrottledThread
{
   private final String name;
   private final Throttler throttler;
   private final ExceptionHandler exceptionHandler;
   private final boolean runAsDaemon;
   private final RunnableThatThrows runnable;

   private Thread thread;
   private boolean running = false;

   public PausableThrottledThread(String name, double runFrequency, RunnableThatThrows runnable)
   {
      this(name, runFrequency, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, false, runnable);
   }

   public PausableThrottledThread(String name, double runFrequency, ExceptionHandler exceptionHandler, RunnableThatThrows runnable)
   {
      this(name, runFrequency, exceptionHandler, false, runnable);
   }

   public PausableThrottledThread(String name, double runFrequency, ExceptionHandler exceptionHandler, boolean runAsDaemon, RunnableThatThrows runnable)
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
      running = true;
      thread = new Thread(() ->
      {
         try
         {
            while (running)
            {
               throttler.waitAndRun();

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

   public Thread getThread()
   {
      return thread;
   }
}
