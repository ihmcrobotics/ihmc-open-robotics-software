package us.ihmc.utilities.ros.subscriber;


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
}
