package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

/**
 * A thread that executes code in a loop.
 * Optionally, you may set a limit to the loop frequency.
 * This thread does not finish running until {@link #destroy()} or {@link #blockingDestroy()} is called.
 * Once started, make sure to destroy this thread.
 */
// TODO: Update and finish comments
public class PausableLoopingThread extends Thread
{
   private static final int PAUSE = 0;
   private static final int LOOP_INDEFINITELY = -1;

   private final RunnableThatThrows runnableThatThrows;
   private final ExceptionHandler exceptionHandler;

   /*
    * Counter for number of time to run the loop.
    * The counter is decremented each time the loop runs.
    * 0 = pause (don't run until counter value is changed).
    * -1 = loop indefinitely (keep looping until told otherwise).
    */
   private volatile int remainingRunCounter = 0;
   private final Object runLock = new Object();

   /*
    * Indicates whether this object is destroyed.
    * The loop will come to a finish when isDestroyed == true.
    * Does not equal to Thread.isAlive(), as the thread may take
    * some time to finish executing after isDestroyed becomes true.
    */
   private volatile boolean isDestroyed = false;

   /* Throttler for optionally set loop period/frequency limit */
   private final Throttler throttler = new Throttler();
   private volatile double loopPeriodLowerLimit = -1.0;

   public PausableLoopingThread(String name)
   {
      this(-1.0, name);
   }

   public PausableLoopingThread(double loopFrequencyLimit, String name)
   {
      this(DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, loopFrequencyLimit, name);
   }

   public PausableLoopingThread(ExceptionHandler exceptionHandler, double loopFrequencyLimit, String name)
   {
      this(null, exceptionHandler, loopFrequencyLimit, name);
   }

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
      super(name);
      super.start();
      this.runnableThatThrows = runnableThatThrows;
      this.exceptionHandler = exceptionHandler;
      limitLoopFrequency(loopFrequencyLimit);
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
   @Override
   public void start()
   {
      setNumberOfRuns(LOOP_INDEFINITELY);
   }

   /**
    * Signal the thread to execute the loop once, then pause.
    * The number of remaining runs is overridden to 1, regardless of its previous value.
    */
   public void runOnce()
   {
      setNumberOfRuns(1);
   }

   /**
    * Pauses re-executing the passed in {@link RunnableThatThrows} upon its completion.
    */
   public void pause()
   {
      setNumberOfRuns(PAUSE);
   }

   public void setNumberOfRuns(int numberOfTimesToRun)
   {
      synchronized (runLock)
      {
         remainingRunCounter = numberOfTimesToRun;
         runLock.notify();
      }
   }

   public int getRemainingRuns()
   {
      return remainingRunCounter;
   }

   public boolean isDestroyed()
   {
      return isDestroyed;
   }

   public boolean isLooping()
   {
      int remainingRuns = getRemainingRuns();
      return !isDestroyed && (remainingRuns > 0 || remainingRuns == LOOP_INDEFINITELY);
   }

   /**
    * Signals the thread to stop once passed in {@link RunnableThatThrows}
    * finishes executing for the last time.
    * The thread cannot be re-started after calling this method.
    */
   public void destroy()
   {
      synchronized (runLock)
      {
         isDestroyed = true;
         runLock.notify();
      }
   }

   /**
    * Signals the thread to stop once the passed in {@link RunnableThatThrows}
    * finishes executing and waits until the thread exits.
    * Same as calling {@link #destroy()} then {@link #join()}.
    * InterruptedExceptions are ignored. To handle interrupted exceptions,
    * call {@link #destroy()} then {@link #join()} manually.
    */
   public void blockingDestroy()
   {
      destroy();
      try
      {
         join();
      }
      catch (InterruptedException ignored) {}
   }

   /**
    * TODO: Finish this comment
    * @throws Throwable
    */
   protected void runInLoop() throws Throwable
   {
      if (runnableThatThrows != null)
         runnableThatThrows.run();
   }

   /**
    * <p>
    * The {@link Thread#run()} method, overridden to run in a loop.
    * To extend this class {@link Thread} style, override {@link #runInLoop()} instead.
    * </p>
    * <p>
    * DO NOT CALL THIS METHOD. Well, you can, but why would you?
    * You are using a thread to run things asynchronously, but calling this would run the loop synchronously.
    * Why would you want that?
    * </p>
    * <p>
    * This method is a necessary evil committed for this class to extend Thread.
    * </p>
    */
   @Override
   public final void run()
   {
      // Run while the thread is not finalizing
      while (!isDestroyed)
      {
         try
         {
            synchronized (runLock)
            {  // No more runs remaining -> wait until something changes
               if (remainingRunCounter == 0)
               {
                  runLock.wait();
                  continue;
               }

               // Decrement the counter for the run that's about to occur
               if (remainingRunCounter > 0)
                  remainingRunCounter--;
            }

            // If a period/frequency limit was set, wait until loop can run.
            if (loopPeriodLowerLimit > 0.0)
            {
               /*
                * This guy must not swallow interrupts.
                * As of writing this comment, LockSupport.parkNanos() is used internally to block.
                * Although the throttler will block until the period has elapsed, the thread
                * remain interrupted.
                */
               throttler.waitAndRun(loopPeriodLowerLimit);
            }
         }
         catch (InterruptedException interrupted)
         {  // Maintain interrupted status so that runInLoop can handle it
            interrupt();
         }

         ExceptionTools.handle(this::runInLoop, exceptionHandler);
      }
   }
}
