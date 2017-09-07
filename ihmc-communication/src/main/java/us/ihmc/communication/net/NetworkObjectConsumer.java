package us.ihmc.communication.net;

public interface NetworkObjectConsumer<T>
{
   public abstract void sendPacketToNetworkProcessor(T object, int byteSize);
}
