package us.ihmc.communication.net;

public interface NetworkObjectConsumer<T>
{
   public abstract void consumeObject(T object, int byteSize);
}
