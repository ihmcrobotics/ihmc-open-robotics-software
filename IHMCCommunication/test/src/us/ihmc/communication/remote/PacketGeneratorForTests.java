package us.ihmc.communication.remote;

import java.io.Serializable;

public interface PacketGeneratorForTests<T extends Serializable>
{
   public abstract T generatePacket();
}
