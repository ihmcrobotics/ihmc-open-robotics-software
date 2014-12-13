package us.ihmc.communication.net;

public interface GlobalObjectConsumer extends ObjectConsumer<Object>
{
   public void consumeObject(Object object, boolean consumeGlobal);
}
