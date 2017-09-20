package us.ihmc.tools.thread;


/*
 * PeriodicWorker. A class for doing some work each tick. Can be used in either a real time fashion or a 
 * non-real-time fashion.
 */
public abstract class PeriodicWorker
{
   private boolean cycle = true;
   private boolean stopped = false;
   
   public abstract void doWorkCycle();
   public abstract String getDescription();
   
   public void doWork()
   {
      if (cycle == true)
      {
         doWorkCycle();
      }
      else
      {
         stopped = true;
      }
   }
   
   public void requestStop()
   {
      cycle = false;
   }

   public boolean isStopped()
   {
      return this.stopped;
   }
}
