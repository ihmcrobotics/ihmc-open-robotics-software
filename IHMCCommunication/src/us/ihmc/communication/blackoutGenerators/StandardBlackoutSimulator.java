package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.ReentrantLock;

public abstract class StandardBlackoutSimulator implements CommunicationBlackoutSimulator
{
   private final ReentrantLock lock = new ReentrantLock();
   private CommunicationBlackoutGenerator blackoutGenerator;
   private volatile boolean enableBlackouts = false;
   private volatile boolean blackout = false;
   private ExecutorService executor = Executors.newSingleThreadExecutor();
   
   public StandardBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator)
   {
      this.blackoutGenerator = blackoutGenerator;
   }
   
   public CommunicationBlackoutGenerator getBlackoutGenerator()
   {
      return blackoutGenerator;
   }
   
   @Override
   public void startBlackoutSimulator()
   {
      enableBlackouts = true;
      executor.execute(new BlackoutTimer(getCurrentTime()));
   }
   
   @Override
   public void stopBlackoutSimulator()
   {
      enableBlackouts = false;
   }
   
   @Override
   public boolean blackoutCommunication()
   {
      return blackout;
   }
   
   public void lock()
   {
      lock.lock();
   }
   
   public void unlock()
   {
      lock.unlock();
   }

   class BlackoutTimer implements Runnable
   {
      private long currentBlackoutStartTime;
      private long currentBlackoutLength;
      private long generatorStartTime;
      
      public BlackoutTimer(long currentTime)
      {
         generatorStartTime = currentTime;
      }
      
      @Override
      public void run()
      {
         currentBlackoutStartTime = generatorStartTime;
         currentBlackoutLength = StandardBlackoutSimulator.this.getBlackoutGenerator().calculateNextBlackoutLength(generatorStartTime);

         while(enableBlackouts)
         {
            StandardBlackoutSimulator.this.lock();
            if(getCurrentTime() % 1000 == 0)
               System.out.println(getCurrentTime() + " " + blackout);
            long currentTime = StandardBlackoutSimulator.this.getCurrentTime();
            
            if(currentTime - currentBlackoutStartTime < 0)
               blackout = false;
            else if(currentTime - currentBlackoutStartTime >= currentBlackoutLength)
            {
               blackout = false;
               currentBlackoutStartTime = currentTime + 1000;
               currentBlackoutLength = StandardBlackoutSimulator.this.getBlackoutGenerator().calculateNextBlackoutLength(currentTime - generatorStartTime);
            }
            else
               blackout = true;
            
            StandardBlackoutSimulator.this.unlock();
         }
      }
   }
}
