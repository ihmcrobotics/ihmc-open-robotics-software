package us.ihmc.utilities.ros.publisher;

import org.ros.internal.message.Message;
import org.ros.node.topic.Publisher;

public abstract class RosTopicPublisher<T extends Message>
{
   private final String messageType;
   private final boolean latched;
   private Publisher<T> publisher;
   private T message;
   

   public RosTopicPublisher(String messageType, boolean latched)
   {
      this.messageType = messageType;
      this.latched = latched;
   }

   public String getMessageType()
   {
      return messageType;
   }
   
   public void setPublisher(Publisher<T> publisher)
   {
      this.publisher = publisher;
      this.message = publisher.newMessage();
      
      this.publisher.setLatchMode(latched);
   }
   
   protected void publish(T message)
   {
      checkInitialized();
      publisher.publish(message);
   }
   
   protected T getMessage()
   {
      checkInitialized();
      return message;
   }
   
   private void checkInitialized()
   {
      if(publisher == null)
      {
         throw new RuntimeException("RosTopicPublisher is not registered with RosMainNode");
      }
   }

   public void connected()
   {
   }
}
