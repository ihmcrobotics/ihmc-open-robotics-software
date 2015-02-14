package us.ihmc.utilities.ros.subscriber;

import org.ros.node.topic.Subscriber;


public abstract class AbstractRosTopicSubscriber<T> implements RosTopicSubscriberInterface<T>
{
   private final String messageType;
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
      
   }
   

   private Object lock = new Object();
   private boolean isRegistered = false;

   public void registered(Subscriber<T> subscriber)
   {
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
