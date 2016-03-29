package us.ihmc.communication.packets;

import java.util.List;

public interface MultiplePacketHolder
{
   public abstract void parseMessageIdToChildren();

   public abstract List<Packet<?>> getPackets();
}
