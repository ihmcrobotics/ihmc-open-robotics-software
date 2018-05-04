package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StateEstimatorModePacket;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;

public class StateEstimatorModeSubscriber implements PacketConsumer<StateEstimatorModePacket>
{
   private final AtomicReference<StateEstimatorMode> referenceToRequestedMode = new AtomicReference<StateEstimatorMode>(null);

   public StateEstimatorModeSubscriber()
   {
   }

   public boolean checkForNewOperatingModeRequest()
   {
      return referenceToRequestedMode.get() != null;
   }

   public StateEstimatorMode getRequestedOperatingMode()
   {
      return referenceToRequestedMode.getAndSet(null);
   }

   @Override
   public void receivedPacket(StateEstimatorModePacket packet)
   {
      if (packet == null)
         return;

      StateEstimatorMode requestedOperatingMode = StateEstimatorMode.fromByte(packet.getRequestedStateEstimatorMode());
      if (requestedOperatingMode != null)
      {
         referenceToRequestedMode.set(requestedOperatingMode);
      }
   }

}
