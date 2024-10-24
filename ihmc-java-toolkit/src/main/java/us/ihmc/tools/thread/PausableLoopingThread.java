package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A thread that executes code in a loop.
 * Optionally, you may set a limit to the loop frequency.
 * This thread does not finish running until {@link #destroy()} or {@link #blockingDestroy()} is called.
 * Once started, make sure to destroy this thread.
 */
public class PausableLoopingThread
{
   private enum RunCommand
   {
      PAUSE, LOOP, RUN_ONCE, FINALIZE
   }

   private final AtomicReference<RunCommand> runCommand = new AtomicReference<>(RunCommand.PAUSE);

   private final RunnableThatThrows runnableThatThrows;
   private final ExceptionHandler exceptionHandler;

   private final Thread loopThread;
   private final Throttler throttler = new Throttler();
   private volatile double loopPeriodLowerLimit = -1.0;

   public PausableLoopingThread(RunnableThatThrows runnableThatThrows, String name)
   {
      this(runnableThatThrows, -1.0, name);
   }

   public PausableLoopingThread(RunnableThatThrows runnableThatThrows, double loopFrequencyLimit, String name)
   {
      this(runnableThatThrows, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, loopFrequencyLimit, name);
   }

   public PausableLoopingThread(RunnableThatThrows runnableThatThrows, ExceptionHandler exceptionHandler, String name)
   {
      this(runnableThatThrows, exceptionHandler, -1.0, name);
   }

   public PausableLoopingThread(RunnableThatThrows runnableThatThrows, ExceptionHandler exceptionHandler, double loopFrequencyLimit, String name)
   {
      this.runnableThatThrows = runnableThatThrows;
      this.exceptionHandler = exceptionHandler;
      limitLoopFrequency(loopFrequencyLimit);
      loopThread = new Thread(this::loop, name);
      loopThread.start();
   }

   /**
    * Limit the frequency of the execution loop.
    * To un-limit the loop frequency, pass in a number less than or equal to 0.0.
    * @param frequencyLimit The limit for the loop frequency.
    *                       If zero or negative, the loop's frequency is not limited.
    */
   public void limitLoopFrequency(double frequencyLimit)
   {
      loopPeriodLowerLimit = Conversions.hertzToSeconds(frequencyLimit);
   }

   /**
    * Starts executing the passed in {@link RunnableThatThrows} in a loop.
    */
   public void start()
   {
      synchronized (runCommand)
      {
         runCommand.set(RunCommand.LOOP);
         runCommand.notify();
      }
   }

   /**
    * Signal the thread to execute the passed in {@link RunnableThatThrows} method once, then pause.
    */
   public void runOnce()
   {
      synchronized (runCommand)
      {
         runCommand.set(RunCommand.RUN_ONCE);
         runCommand.notify();
      }
   }

   /**
    * Pauses re-executing the passed in {@link RunnableThatThrows} upon its completion.
    */
   public void pause()
   {
      runCommand.set(RunCommand.PAUSE);
   }

   /**
    * Signals the thread to stop once passed in {@link RunnableThatThrows}
    * finishes executing for the last time.
    * The thread cannot be re-started after calling this method.
    */
   public void destroy()
   {
      synchronized (runCommand)
      {
         runCommand.set(RunCommand.FINALIZE);
         runCommand.notify();
      }
   }

   /**
    * Signals the thread to stop once the passed in {@link RunnableThatThrows}
    * finishes executing and waits until the thread exits.
    * Same as calling {@link #destroy()} then joining.
    */
   public void blockingDestroy()
   {
      destroy();
      try
      {
         loopThread.join();
      }
      catch (InterruptedException ignored) {}
   }

   private void loop()
   {  // Run while the thread is not finalizing
      while (runCommand.get() != RunCommand.FINALIZE)
      {
         try
         {
            synchronized (runCommand)
            {
               RunCommand currentCommand = runCommand.get();
               if (currentCommand == RunCommand.PAUSE)
               {  // Currently paused
                  runCommand.wait();
                  continue;
               }
               else if (currentCommand == RunCommand.RUN_ONCE)
                  pause(); // Set the run command to pause after this loop
            }

            if (loopPeriodLowerLimit > 0.0)
               throttler.waitAndRun(loopPeriodLowerLimit);
         }
         catch (InterruptedException interrupted)
         {  // Maintain interrupted status so that runInLoop can handle it
            loopThread.interrupt();
         }

         ExceptionTools.handle(runnableThatThrows, exceptionHandler);
      }
   }

   public boolean isLooping()
   {
      return runCommand.get() == RunCommand.LOOP;
   }

   /**
    * <p>
    * Although this class represents a thread, it does not extend {@link Thread}.
    * Instead, it contains an internal {@link Thread} that runs the thread
    * represented by this class.
    * Use this method to access the {@link Thread}` running the loop.
    * </p>
    * <p>
    * Interrupting the internal thread will relay the interrupt to the passed in {@link RunnableThatThrows}.
    * </p>
    *
    * @return The internal {@link Thread} running the loop.
    */
   public Thread getInternalThread()
   {
      return loopThread;
   }
}
