package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.MathTools;

public class DesiredComHeightProvider implements PacketConsumer<ComHeightPacket>
{
   // Do not do it like this, preferably use one atomic
   private final AtomicBoolean newDataAvailable = new AtomicBoolean(false);
   private final AtomicDouble comHeightOffset = new AtomicDouble(0.0);
   private final AtomicDouble trajectoryTime = new AtomicDouble(0.0);

   private final double defaultTrajectoryTime = 0.5; //Hackish default time for height trajectory. We need to just ensure that this is always set in the packet instead and then get rid of this.
   private final HumanoidGlobalDataProducer globalDataProducer;

   public DesiredComHeightProvider(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
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

   @Override
   public void receivedPacket(ComHeightPacket packet)
   {
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateCoMHeightPacket(packet);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(ComHeightPacket.class, errorMessage);
            return;
         }
      }

      newDataAvailable.set(true);

      double heightOffset = packet.getHeightOffset();
      heightOffset = MathTools.clipToMinMax(heightOffset, ComHeightPacket.MIN_COM_HEIGHT, ComHeightPacket.MAX_COM_HEIGHT);
      comHeightOffset.set(heightOffset);
      double packetTime = packet.getTrajectoryTime();

      if ((packetTime < 1e-7) || (Double.isNaN(packetTime) || packetTime > 1000))
      {
         packetTime = defaultTrajectoryTime;
      }
      trajectoryTime.set(packetTime);
   }
}
