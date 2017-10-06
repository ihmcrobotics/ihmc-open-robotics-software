package us.ihmc.tools.thread;

public class NonRealTimePeriodicThread implements Runnable
{
   private final PeriodicWorker periodicWorker;
   private final long sleepTime;
   
   public NonRealTimePeriodicThread(PeriodicWorker periodicWorker, long sleepTime)
   {
      this.periodicWorker = periodicWorker;
      this.sleepTime = sleepTime;
   }
   
   public void createAndStartThread()
   {
      Thread thread = new Thread(this);
      thread.start();
   }
   
   @Override
   public void run()
   {
      while (!periodicWorker.isStopped())
      {
         periodicWorker.doWork();

         try
         {
            Thread.sleep(sleepTime);
         } 
         catch (InterruptedException e)
         {
         }
      }      
   }
}
