package us.ihmc.utilities.ros.publisher;

import org.ros.internal.message.Message;
import org.ros.node.topic.Publisher;

public abstract class RosTopicPublisher<T extends Message>
{
   private final String messageType;
   private final boolean latched;
   private Publisher<T> publisher;

   private final Object syncObject = new Object();
   private boolean isRegistered = false;
   
   public RosTopicPublisher(String messageType, boolean latched)
   {
      this.messageType = messageType;
      this.latched = latched;
   }

   public String getMessageType()
   {
      return messageType;
   }

   public void registered(Publisher<T> publisher)
   {
      this.publisher = publisher;
      this.publisher.setLatchMode(latched);

      synchronized (syncObject)
      {
         isRegistered = true;
         syncObject.notify();
      }
   }
   

   public void waitTillRegistered()
   {
      while (!isRegistered)
      {
         synchronized (syncObject)
         {
            try
            {
               syncObject.wait();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }
   }

   protected void publish(T message)
   {
      checkInitialized();
      publisher.publish(message);
   }

   protected T getMessage()
   {
      checkInitialized();

      return publisher.newMessage();
   }

   private void checkInitialized()
   {
      if (publisher == null)
      {
         throw new RuntimeException("RosTopicPublisher is not registered with RosMainNode");
      }
   }

   public boolean isConnected()
   {
      return publisher != null;
   }

   public void connected()
   {
   }


}
