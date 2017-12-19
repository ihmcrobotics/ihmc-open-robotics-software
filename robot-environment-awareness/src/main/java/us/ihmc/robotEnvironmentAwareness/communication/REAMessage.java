package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.APIElementId;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;

public final class REAMessage<T> extends Packet<REAMessage<T>>
{
   public APIElementId topicId;
   public Object messageContent;

   public REAMessage()
   {
   }

   public REAMessage(Topic<T> topic, T messageContent)
   {
      this.topicId = topic.getUniqueId();
      this.messageContent = messageContent;
   }

   public APIElementId getTopicId()
   {
      return topicId;
   }

   public Object getMessageContent()
   {
      return messageContent;
   }

   @Override
   public boolean epsilonEquals(REAMessage<T> other, double epsilon)
   {
      return topicId.equals(other.topicId) && messageContent.equals(other.messageContent);
   }
}
