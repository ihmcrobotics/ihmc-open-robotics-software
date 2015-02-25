package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public class SystemTimeBasedBlackoutSimulator extends StandardBlackoutSimulator
{
   private final boolean DEBUG = false;
   
   public SystemTimeBasedBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator, PacketCommunicator packetCommunicator)
   {
      super(blackoutGenerator, packetCommunicator);
      
      if(DEBUG)
         Executors.newSingleThreadExecutor().execute(new BlackoutTimerWatcher());
   }

   @Override
   public long getCurrentTime(TimeUnit timeUnit)
   {
      return timeUnit.convert(System.currentTimeMillis(), TimeUnit.MILLISECONDS);
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
               System.out.println(getCurrentTime(TimeUnit.MILLISECONDS));
            
            inBlackout = SystemTimeBasedBlackoutSimulator.this.blackoutCommunication();
         }
      }
   }
}
