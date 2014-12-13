package us.ihmc.communication.net;

import java.util.ArrayList;

public abstract class ObjectProducer<T>
{
   private final ArrayList<ObjectConsumer<? super T>> consumers = new ArrayList<ObjectConsumer<? super T>>();
   
   public void addConsumer(ObjectConsumer<? super T> consumer)
   {
      consumers.add(consumer);
   }
   
   protected void sendObject(T object)
   {
      for(ObjectConsumer<? super T> consumer : consumers)
      {
         consumer.consumeObject(object);
      }
   }

}
