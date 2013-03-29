package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import org.ros.message.MessageListener;

public abstract class RosTopicSubscriber<T> implements MessageListener<T>
{
   private final String messageType;
   public RosTopicSubscriber(String messageType)
   {
      this.messageType = messageType;
   }
   
   public String getMessageType()
   {
      return messageType;
   }
}
