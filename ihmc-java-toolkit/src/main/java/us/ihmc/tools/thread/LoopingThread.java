package us.ihmc.tools.thread;

import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.ExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A thread that executes code in a loop.
 * Optionally, you may set a limit to the loop frequency.
 * This thread does not finish running until {@link #close()} or {@link #blockingClose()} is called.
 * Once started, make sure to close this thread.
 */
public class LoopingThread extends Thread
{
   private final Runnable target;
   private Throttler throttler;
   private final AtomicReference<RunCommand> runCommand = new AtomicReference<>(RunCommand.INITIALIZE);

   public LoopingThread() {
      super();
      target = null;
   }

   public LoopingThread(double loopFrequencyLimit)
   {
      this();
      limitLoopFrequency(loopFrequencyLimit);
   }

   public LoopingThread(Runnable target, String name)
   {
      super(target, name);
      this.target = target;
   }

   public LoopingThread(double loopFrequencyLimit, String name) {
      this(null, loopFrequencyLimit, name);
   }

   public LoopingThread(Runnable target, double loopFrequencyLimit, String name)
   {
      this(target, name);
      limitLoopFrequency(loopFrequencyLimit);
   }

   public LoopingThread(RunnableThatThrows targetThatThrows, ExceptionHandler exceptionHandler, String name)
   {
      super(() -> ExceptionTools.handle(targetThatThrows, exceptionHandler), name);
      target = () -> ExceptionTools.handle(targetThatThrows, exceptionHandler);
   }

   public LoopingThread(RunnableThatThrows targetThatThrows, ExceptionHandler exceptionHandler, double loopFrequencyLimit, String name)
   {
      this(targetThatThrows, exceptionHandler, name);
      limitLoopFrequency(loopFrequencyLimit);
   }

   public void limitLoopFrequency(double frequencyLimit)
   {
      if (throttler == null)
         throttler = new Throttler();

      throttler.setFrequency(frequencyLimit);
   }

   /**
    * Starts executing the {@link #runInLoop()} method in a loop.
    */
   @Override
   public void start()
   {
      synchronized (runCommand)
      {
         initializeIfNeeded();

         runCommand.set(RunCommand.LOOP);
         runCommand.notify();
      }
   }

   /**
    * Signal the thread to execute the {@link #runInLoop()} method once, then pause.
    */
   public void runOnce()
   {
      synchronized (runCommand)
      {
         initializeIfNeeded();

         runCommand.set(RunCommand.RUN_ONCE);
         runCommand.notify();
      }
   }

   private void initializeIfNeeded()
   {
      if (runCommand.compareAndSet(RunCommand.INITIALIZE, RunCommand.PAUSE))
         super.start();
   }

   /**
    * Pauses re-executing the {@link #runInLoop()} method upon its completion.
    */
   public void pause()
   {
      runCommand.set(RunCommand.PAUSE);
   }

   /**
    * Finalizes the execution of {@link #runInLoop()} and stops the thread.
    * The thread cannot be re-started after calling this method.
    */
   public void close()
   {
      synchronized (runCommand)
      {
         runCommand.set(RunCommand.FINALIZE);
         runCommand.notify();
      }
   }

   /**
    * Signals the thread to finalize execution of {@link #runInLoop()}
    * and waits until the thread exits.
    * Same as calling {@link #close()} then {@link #join()}.
    */
   public void blockingClose()
   {
      close();
      try
      {
         join();
      }
      catch (InterruptedException ignored) {}
   }

   /**
    * The method that is executed in a loop.
    * You may either provide this class a
    * {@link Runnable}/{@link RunnableThatThrows} to run,
    * or override this method.
    */
   public void runInLoop()
   {
      if (target != null)
         target.run();
   }

   @Override
   public final void run()
   {  // Run while the thread is not finalizing
      while (runCommand.get() != RunCommand.FINALIZE)
      {
         synchronized (runCommand)
         {
            RunCommand currentCommand = runCommand.get();
            if (currentCommand == RunCommand.PAUSE)
            {  // Currently paused
               try
               {  // Wait until we're notified to run
                  runCommand.wait();
               }
               catch (InterruptedException ignored) {}
               continue;
            }
            else if (currentCommand == RunCommand.RUN_ONCE)
               pause(); // Set the run command to pause after this loop
         }

         if (throttler != null)
            throttler.waitAndRun();

         runInLoop();
      }
   }

   public boolean isLooping()
   {
      return runCommand.get() == RunCommand.LOOP;
   }

   private enum RunCommand
   {
      INITIALIZE, LOOP, PAUSE, RUN_ONCE, FINALIZE
   }
}
