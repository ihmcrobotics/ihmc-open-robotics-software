package us.ihmc.commonWalkingControlModules.packetProviders;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStatePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;

public class DesiredHighLevelStateProvider implements PacketConsumer<HighLevelStatePacket>
{
   private final AtomicReference<HighLevelState> highLevelState = new AtomicReference<HighLevelState>(null);
   
   public DesiredHighLevelStateProvider()
   {
   }
   
   public boolean checkForNewState()
   {
      return highLevelState.get() != null;
   }
   
   public HighLevelState getDesiredHighLevelState()
   {
      return highLevelState.getAndSet(null);
   }
   
   public void receivedPacket(HighLevelStatePacket object)
   {
       highLevelState.set(object.getHighLevelState());
   }

}
