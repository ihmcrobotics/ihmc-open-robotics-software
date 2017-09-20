package us.ihmc.communication.net;

public interface ObjectConsumer<T>
{
   public abstract void consumeObject(T object);
}
