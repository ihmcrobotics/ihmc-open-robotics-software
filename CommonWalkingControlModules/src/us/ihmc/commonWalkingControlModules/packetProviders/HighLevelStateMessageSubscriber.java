package us.ihmc.commonWalkingControlModules.packetProviders;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

public class HighLevelStateMessageSubscriber implements PacketConsumer<HighLevelStateMessage>
{
   private final AtomicReference<HighLevelState> highLevelState = new AtomicReference<HighLevelState>(null);

   public HighLevelStateMessageSubscriber()
   {
   }

   public boolean isNewMessageAvailable()
   {
      return highLevelState.get() != null;
   }

   public HighLevelState pollMessage()
   {
      return highLevelState.getAndSet(null);
   }

   @Override
   public void receivedPacket(HighLevelStateMessage object)
   {
      highLevelState.set(object.getHighLevelState());
   }
}
