package us.ihmc.communication.blackoutGenerators;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.BlackoutPacket;

public abstract class StandardBlackoutSimulator implements CommunicationBlackoutSimulator, PacketConsumer<BlackoutPacket>
{
   private final int GOOD_COMMS_PERIOD_IN_MILLI = 1000;
   
   private final ReentrantLock lock = new ReentrantLock();
   private CommunicationBlackoutGenerator blackoutGenerator;
   private volatile boolean enableBlackouts = false;
   private volatile boolean blackout = false;
   private ExecutorService executor = Executors.newSingleThreadExecutor();
   
   public StandardBlackoutSimulator(CommunicationBlackoutGenerator blackoutGenerator, PacketCommunicator packetCommunicator)
   {
      this.blackoutGenerator = blackoutGenerator;
      packetCommunicator.attachListener(BlackoutPacket.class, this);
   }
   
   public CommunicationBlackoutGenerator getBlackoutGenerator()
   {
      return blackoutGenerator;
   }
   
   @Override
   public void enableBlackouts(boolean enable)
   {
      enableBlackouts = enable;
      
      if(enable)
         executor.execute(new BlackoutTimer());
      else
         blackout = false;
   }
   
   @Override
   public boolean blackoutCommunication()
   {
      return blackout;
   }
   
   @Override
   public boolean isEnabled()
   {
      return enableBlackouts;
   }
   
   @Override
   public void receivedPacket(BlackoutPacket packet)
   {
      enableBlackouts(packet.getBlackout());
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
      
      @Override
      public void run()
      {
         generatorStartTime = StandardBlackoutSimulator.this.getCurrentTime(TimeUnit.MILLISECONDS);
         currentBlackoutStartTime = generatorStartTime;
         currentBlackoutLength = StandardBlackoutSimulator.this.getBlackoutGenerator().calculateNextBlackoutLength(generatorStartTime, TimeUnit.MILLISECONDS);

         while(enableBlackouts)
         {
            StandardBlackoutSimulator.this.lock();
            long currentTime = StandardBlackoutSimulator.this.getCurrentTime(TimeUnit.MILLISECONDS);
            
            if(currentTime - currentBlackoutStartTime < 0)
               blackout = false;
            else if(currentTime - currentBlackoutStartTime >= currentBlackoutLength)
            {
               blackout = false;
               currentBlackoutStartTime = currentTime + GOOD_COMMS_PERIOD_IN_MILLI;
               currentBlackoutLength = StandardBlackoutSimulator.this.getBlackoutGenerator().calculateNextBlackoutLength(currentTime - generatorStartTime, TimeUnit.MILLISECONDS);
            }
            else
               blackout = true;
            
            StandardBlackoutSimulator.this.unlock();
         }
      }
   }
}
