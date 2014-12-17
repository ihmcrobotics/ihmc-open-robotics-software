package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.walking.ComHeightPacket;

import com.google.common.util.concurrent.AtomicDouble;

public class DesiredComHeightProvider
{
   private final ComHeightPacketConsumer comHeightPacketConsumer;

   // Do not do it like this, preferably use one atomic
   private final AtomicBoolean newDataAvailable = new AtomicBoolean(false);
   private final AtomicDouble comHeightOffset = new AtomicDouble(0.0);
   private final AtomicDouble trajectoryTime = new AtomicDouble(0.0);

   private final double defaultTrajectoryTime = 0.5; //Hackish default time for height trajectory. We need to just ensure that this is always set in the packet instead and then get rid of this.
   
   public DesiredComHeightProvider()
   {
      comHeightPacketConsumer = new ComHeightPacketConsumer();
   }

   public ObjectConsumer<ComHeightPacket> getComHeightPacketConsumer()
   {
      return comHeightPacketConsumer;
   }


   private class ComHeightPacketConsumer implements ObjectConsumer<ComHeightPacket>
   {
      public ComHeightPacketConsumer()
      {
      }

      @Override
      public void consumeObject(ComHeightPacket packet)
      {
         newDataAvailable.set(true);
         comHeightOffset.set(packet.getHeightOffset());
         double packetTime = packet.getTrajectoryTime();
         
         if ((packetTime < 1e-7) || (Double.isNaN(packetTime)))
         {
            packetTime = defaultTrajectoryTime;
         }
         trajectoryTime.set(packetTime);
      }
   }


   public boolean isNewComHeightInformationAvailable()
   {
      return newDataAvailable.getAndSet(false);
   }


   public double getComHeightTrajectoryTime()
   {
      return trajectoryTime.get();
   }

   public double getComHeightOffset()
   {
      return comHeightOffset.get();
   }

}
