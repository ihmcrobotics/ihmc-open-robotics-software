package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.Executors;

public class SystemTimeBasedBlackoutSimulator extends StandardBlackoutSimulator
{
   private final boolean DEBUG = false;
   
   public SystemTimeBasedBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator)
   {
      super(blackoutGenerator);
      
      if(DEBUG)
         Executors.newSingleThreadExecutor().execute(new BlackoutTimerWatcher());
   }

   @Override
   public long getCurrentTime()
   {
      return System.currentTimeMillis();
   }
   
   public static void main(String[] args)
   {
      CommunicationBlackoutGenerator blackoutGenerator = new ConstantBlackoutGenerator(3000);
      SystemTimeBasedBlackoutSimulator blackoutSimulator = new SystemTimeBasedBlackoutSimulator(blackoutGenerator);
      blackoutSimulator.startBlackoutSimulator();
   }
   
   class BlackoutTimerWatcher implements Runnable
   {
      private boolean inBlackout; 
      
      @Override
      public void run()
      {
         inBlackout = SystemTimeBasedBlackoutSimulator.this.blackoutCommunication();
         
         while(true)
         {
            if(inBlackout != SystemTimeBasedBlackoutSimulator.this.blackoutCommunication())
               System.out.println(getCurrentTime());
            
            inBlackout = SystemTimeBasedBlackoutSimulator.this.blackoutCommunication();
         }
      }
   }
}
