package us.ihmc.tools.thread;

/**
 * <p>Title: </p> InterruptableThread
 *
 * <p>Description: </p> This class is designed to be a pattern for threads that
 * need to be interruptable (usually time limited).  It also manages the tracking
 * the state during operations.
 *
 */
public abstract class InterruptableWorker implements Runnable
{
   public enum STATE
   {
      UNINITIALIZED, INITIALIZED, WORKING, SUCCESSFULLY_COMPLETED, UNSUCCESSFULLY_COMPLETED, PENDING_INTERRUPTION, INTERRUPTED
   }

   private STATE currentState = STATE.UNINITIALIZED;


   /**
    * This method needs to be overridden and should perform the actual work
    * It should periodically chech the isInterrupted method to know if it should
    * stop. It should return one of the three acceptable finish states:
    *    SUCCESSFULLY_COMPLETED
    *    UNSUCCESSFULLY_COMPLETED
    *    INTERRUPTED
    *
    * @return STATE
    */
   abstract public STATE doWork();

   /**
    * This method needs to be overridden and should be used by the application
    * to get the current result while waiting as well as get the final result
    * once finished
    *
    * @return Object
    */
   abstract public Object getCurrentResult();

   /**
    * This method should be overridden, but should call this method as the first
    * step.
    */
   public void reset()
   {
      setCurrentState(STATE.INITIALIZED);
   }

   /**
    * This method returns the current state
    *
    * @return STATE
    */
   public synchronized STATE getCurrentState()
   {
      return currentState;
   }



   /**
    * This is the normal thread run method called on start
    * It tells the worker to start and tracks the resulting state
    * The worker should return the appropriate finish state:
    *    SUCCESSFULLY_COMPLETED
    *    UNSUCCESSFULLY_COMPLETED
    *    INTERRUPTED
    */
   public void startWorkOnANewThread()
   {
      setCurrentState(STATE.WORKING);
      Thread thread = new Thread(this, "InterruptableWorker");
      thread.setDaemon(true);
      thread.start();
   }

   public void startWorkOnThisThread()
   {
      setCurrentState(STATE.WORKING);
      run();
   }

   @Override
   public void run()
   {
      STATE state = doWork();
      if ((state != STATE.SUCCESSFULLY_COMPLETED) && (state != STATE.UNSUCCESSFULLY_COMPLETED) && (state != STATE.INTERRUPTED))
      {
         throw new RuntimeException("Worker returned an invalid state: " + state
                                    + "(must be STATE.SUCCESSFULLY_COMPLETED || STATE.UNSUCCESSFULLY_COMPLETED || state != STATE.INTERRUPTED)");
      }

      setCurrentState(state);
   }

   public synchronized void setCurrentState(STATE state)
   {
      currentState = state;
   }

   /**
    * This method is used to signal the worker to stop
    * and then wait until worker finishes to return
    */
   public void stopWork()
   {
//    System.out.println("InterruptableWorker: Stopping Work!");
      // This is synchronized to prevent the worker from completing
      // prior to the state being set to PENDING.
      synchronized (this)
      {
         // if working, set to pending, otherwise the worker has probably already finished
         if (currentState == STATE.WORKING)
         {
            setCurrentState(STATE.PENDING_INTERRUPTION);

//          System.out.println("InterruptableWorker: STATE.PENDING_INTERRUPTION!");
         }
      }

      while (currentState == STATE.PENDING_INTERRUPTION)
      {
//       System.out.println("InterruptableWorker: Waiting for work to Stopped!");

         try
         {
            Thread.sleep(5);
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }

//    System.out.println("InterruptableWorker: Work Stopped!");

   }

   /**
    * This method should be used by the application to block until work is finished
    * It will block until the timeout and interrupt the worker if not finished in time
    * It does not guarentee the timeout period because it will depend on how long it takes
    * the worker to finish after being told to stop.
    *
    * @param timeout long in milliseconds
    */
   public void waitForResult(long timeout)
   {
      long startTime = System.currentTimeMillis();
      boolean isTimeRemaining = (System.currentTimeMillis() - startTime) < timeout;

      while (((currentState == STATE.WORKING) || (currentState == STATE.PENDING_INTERRUPTION)) && isTimeRemaining)
      {
         // wait a 10ms (based on standard resolution of system time)
         try
         {
            Thread.sleep(10);
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }

         // recalculate how much time is remaining
         isTimeRemaining = (System.currentTimeMillis() - startTime) < timeout;
      }

      // if we ran out of time, tell worker to stop
      if (currentState == STATE.WORKING)
      {
         stopWork();
      }
   }

   /**
    * This method should be used by the InterruptableWorker to check if it has been interrupted
    * It returns true when stop has been called and stays true until reset
    * @return boolean
    */
   public boolean isInterrupted()
   {
      return ((currentState == STATE.PENDING_INTERRUPTION) || (currentState == STATE.INTERRUPTED));
   }

   public boolean isWorkingOrPendingInterruption()
   {
      return ((currentState == STATE.WORKING) || (currentState == STATE.PENDING_INTERRUPTION));
   }

}
