package us.ihmc.tools.thread;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

/**
 * A thread that executes code in a loop.
 * <p>
 * This thread has 3 states: {@link #PAUSE}, {@link #LOOP_INDEFINITELY}, and looping for a set number of iterations.
 * <p>
 * Unlike {@link Thread}, the {@link PausableLoopingThread} becomes alive upon construction,
 * and is in the {@link #PAUSE} state. In this state, the thread is alive, but not running the loop.
 * Calling {@link #start()} begins running the loop indefinitely.
 * To run the loop a set number of iterations, use {@link #loopNIterations(int)}.
 * <p>
 * This thread does not finish running until {@link #destroy()} or {@link #blockingDestroy()} is called.
 * Once created, be sure to destroy this thread.
 * <p>
 * Optionally, you may set a limit to the loop frequency through {@link #limitLoopFrequency(double)},
 * or by passing the frequency limit as a parameter in the constructor.
 * The loop frequency limit may be changed at any time. To de-limit the loop frequency, set the limit <= 0.0.
 * Setting the frequency limit only guarantees that the loop's frequency will not exceed the limit.
 * It does not guarantee that the loop will run at the set frequency, as the code executed within the loop
 * may be too slow to run at that frequency.
 * <p>
 * Like {@link Thread}, the {@link PausableLoopingThread} may be used in two ways:
 * <ul>
 *    <li>
 *       First, by passing in a runnable (or in this case a {@link RunnableThatThrows}) to the constructor.
 *       This runnable will be called in {@link #runInLoop()} every iteration.
 *    <li>
 *       Second, by {@code @Override}ing the {@link #runInLoop()} method.
 *       The code within {@link #runInLoop()} will run every iteration.
 */
public class PausableLoopingThread extends Thread
{
   public static final int PAUSE = 0;
   public static final int LOOP_INDEFINITELY = -1;

   private final RunnableThatThrows runnableThatThrows;
   private final ExceptionHandler exceptionHandler;

   /**
    * Countdown for number of iterations to run.
    * The counter is decremented each time before the iteration runs.
    * Once the counter hits 0, the loop is paused.
    * <ul>
    *    <li> 0 = pause (don't run the loop until counter value is changed).
    *    <li> -1 = loop indefinitely (keep looping until told otherwise).
    *    <li> N > 0 = run the loop N more iterations.
    */
   private volatile int remainingIterations = 0;
   private final Object runLock = new Object();

   /**
    * Indicates whether this object is destroyed.
    * The loop will come to a finish when {@code isDestroyed == true}.
    * Does not equal to {@link Thread#isAlive()}, as the thread may take
    * some time to finish executing after {@code isDestroyed} becomes true.
    */
   private volatile boolean isDestroyed = false;

   /** Throttler for optionally set loop period/frequency limit */
   private final Throttler throttler = new Throttler();

   /** The optionally set lower limit to the loop period. A zero or negative value indicates no limit */
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
    * Limit the frequency of the loop execution.
    * To un-limit the loop frequency, pass in a number less than or equal to 0.0.
    * <p>
    * Setting the frequency limit only guarantees that the loop's frequency will not exceed the limit.
    * It does not guarantee that the loop will run AT the set frequency, as the code executed within the loop
    * may be too slow to run at that frequency.
    *
    * @param frequencyLimit The limit for the loop frequency.
    *                       If zero or negative, the loop's frequency is not limited.
    */
   public void limitLoopFrequency(double frequencyLimit)
   {
      loopPeriodLowerLimit = Conversions.hertzToSeconds(frequencyLimit);
   }

   /**
    * Signal the thread to loop indefinitely.
    */
   @Override
   public void start()
   {
      loopNIterations(LOOP_INDEFINITELY);
   }

   /**
    * Signal the thread to execute one iteration.
    * The number of remaining iterations is overridden to 1, regardless of its previous value.
    * Same as calling {@code loopNIterations(1)}.
    */
   public void loopOnce()
   {
      loopNIterations(1);
   }

   /**
    * Signal the thread to pause looping once the current iteration finishes.
    */
   public void pause()
   {
      loopNIterations(PAUSE);
   }

   /**
    * Signal the thread to loop for the passed in number of iterations.
    * This overrides the remaining number of iterations, regardless of its previous value.
    * <p>
    * This method also accepts {@link #PAUSE} and {@link #LOOP_INDEFINITELY}.
    * <p>
    * To add or subtract to the number of iterations the thread should loop, use {@link #addIterations(int)}.
    *
    * @param iterationsToLoop The number of iterations the thread should loop after this call.
    */
   public void loopNIterations(int iterationsToLoop)
   {
      synchronized (runLock)
      {
         remainingIterations = iterationsToLoop;
         runLock.notify();
      }
   }

   /**
    * Increments the remaining iterations to loop.
    * Same as calling {@code addIterations(1)}.
    */
   public void incrementIterations()
   {
      addIterations(1);
   }

   /**
    * Add N iterations to the remaining iterations counter.
    * If the thread was paused, adding iterations begins the loop.
    * <p>
    * You may also subtract from the number of remaining iterations by passing in a negative number.
    * If the resulting number of iterations is 0, the loop will be paused.
    * Note that this method cannot cause the remaining iteration count to go below 0.
    * <p>
    * This method does not do anything if the thread is looping indefinitely.
    *
    * @param iterationsToAdd The number of iterations to add. This can be a negative value for subtraction.
    */
   public void addIterations(int iterationsToAdd)
   {
      synchronized (runLock)
      {
         // If looping indefinitely, do nothing
         if (remainingIterations < 0)
            return;

         // Add to the remaining iterations counter
         remainingIterations += iterationsToAdd;

         // Ensure remaining iterations counter doesn't become negative in case of subtraction
         if (remainingIterations < 0)
            remainingIterations = 0;

         runLock.notify();
      }
   }

   /**
    * Get the remaining number of iterations this thread plans to run.
    *
    * @return The remaining number of loops this thread plans to run.
    */
   public int getRemainingIterations()
   {
      return remainingIterations;
   }

   /**
    * Whether this thread has been {@link #destroy()}ed.
    * <p>
    * The returned value of this method does not necessarily equate to {@link #isAlive()},
    * as the loop may take some time to finish after the call to {@link #destroy()},
    * during which the thread remains alive.
    *
    * @return {@code true} if {@link #destroy()} or {@link #blockingDestroy()} has been called. {@code false} otherwise.
    */
   public boolean isDestroyed()
   {
      return isDestroyed;
   }

   /**
    * Whether this thread is looping.
    * That is, whether this thread is not paused or destroyed ({@code !(paused || destroyed)}).
    *
    * @return {@code true} if the thread is looping. {@code false} if the thread is paused or destroyed.
    */
   public boolean isLooping()
   {
      int remainingRuns = getRemainingIterations();
      return !((remainingRuns == PAUSE) || isDestroyed);
   }

   /**
    * Signals the thread to stop once the current iteration finishes running.
    * The loop will be exited, and the thread will die.
    * The thread cannot be restarted after calling this method.
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
    * Signals the thread to stop once the current iteration finishes running.
    * The loop will be exited, and the thread will die.
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
    * The method that is executed in a loop.
    * <p>
    * You may {@code @Override} this method with the code to run in the loop.
    * Alternatively, if a {@link RunnableThatThrows} was passed it, this method
    * will call the run method in the loop.
    *
    * @throws Throwable Any throwable that the overriding code or the passed in runnable may throw.
    *       This throwable will be handled by the passed in {@link ExceptionHandler}
    *       (by default it is {@link DefaultExceptionHandler#MESSAGE_AND_STACKTRACE}).
    */
   protected void runInLoop() throws Throwable
   {
      if (runnableThatThrows != null)
         runnableThatThrows.run();
   }

   /**
    * The {@link Thread#run()} method, overridden to run in a loop.
    * To extend this class {@link Thread} style, override {@link #runInLoop()} instead.
    * <p>
    * DO NOT CALL THIS METHOD. Well, you can, but why would you?
    * You are using a thread to run things asynchronously, but calling this would run the loop synchronously.
    * Why would you want that?
    * <p>
    * This method is a necessary evil committed for this class to extend Thread.
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
               if (remainingIterations == 0)
               {
                  runLock.wait();
                  continue;
               }

               // Decrement the counter for the run that's about to occur
               if (remainingIterations > 0)
                  remainingIterations--;
            }

            // If a period/frequency limit was set, wait until loop can run.
            if (loopPeriodLowerLimit > 0.0)
            {
               /*
                * This call must not swallow interrupts.
                * As of writing this comment (Oct, 2024), LockSupport.parkNanos() is used internally to block.
                * Although the throttler will block until the period has elapsed, the thread
                * remains interrupted.
                */
               throttler.waitAndRun(loopPeriodLowerLimit);
            }
         }
         catch (InterruptedException interrupted)
         {  // Maintain interrupted status so that runInLoop can handle it
            interrupt();
         }

         // Run the runInLoop method, and handle any exception it may throw.
         ExceptionTools.handle(this::runInLoop, exceptionHandler);
      }
   }
}
