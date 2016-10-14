package us.ihmc.communication.packets;

public abstract class StatusPacket<T extends StatusPacket<T>> extends Packet<T>
{
   public abstract void set(T other);

}
