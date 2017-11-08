package us.ihmc.communication.packets;

public abstract class RequestPacket<T extends RequestPacket<T>> extends TrackablePacket<T>
{
   public abstract void set(T other);
}
