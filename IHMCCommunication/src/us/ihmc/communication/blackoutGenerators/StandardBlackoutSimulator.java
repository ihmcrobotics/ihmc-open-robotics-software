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
   
   public StandardBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator)
   {
      this.blackoutGenerator = blackoutGenerator;
   }
   
   public void enableBlackouts(boolean enable)
   {
      enableBlackouts = enable;
   }
   
   public CommunicationBlackoutGenerator getBlackoutGenerator()
   {
      return blackoutGenerator;
   }
   
   public void startBlackoutSimulator(long currentTime)
   {
      enableBlackouts = true;
      
      ExecutorService executor = Executors.newSingleThreadExecutor();
      executor.execute(new BlackoutTimer(currentTime));
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
            
            long currentTime = StandardBlackoutSimulator.this.getCurrentTime();
            if(currentTime - currentBlackoutStartTime >= currentBlackoutLength)
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
