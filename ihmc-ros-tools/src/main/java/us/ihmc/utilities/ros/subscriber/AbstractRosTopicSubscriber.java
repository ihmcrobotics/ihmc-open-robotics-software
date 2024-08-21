package us.ihmc.utilities.ros.subscriber;

import org.ros.node.topic.Subscriber;
import us.ihmc.log.LogTools;

public abstract class AbstractRosTopicSubscriber<T> implements RosTopicSubscriberInterface<T>
{
   private final String messageType;
   private final Object lock = new Object();
   private boolean isRegistered = false;

   public AbstractRosTopicSubscriber(String messageType)
   {
      this.messageType = messageType;
   }

   public String getMessageType()
   {
      return messageType;
   }

   public void connected()
   {
      LogTools.info("Connected: {}", messageType);
   }

   public void registered(Subscriber<T> subscriber)
   {
//      LogTools.info("Registered: {} ({})", subscriber.getTopicName().toString(), messageType);
      synchronized (lock)
      {
         isRegistered = true;
         lock.notify();
      }
   }

   public void wailTillRegistered()
   {
      try
      {
         synchronized (lock)
         {
            while(!isRegistered)
               lock.wait();
         }
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}
