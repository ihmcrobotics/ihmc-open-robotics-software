package us.ihmc.robotEnvironmentAwareness.communication;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotEnvironmentAwareness.communication.MessagerAPIFactory.TopicID;

public final class REAMessage<T> extends Packet<REAMessage<T>>
{
   public TopicID topicId;
   public Object messageContent;

   public REAMessage()
   {
   }

   @Override
   public void set(REAMessage<T> other)
   {
      topicId = other.topicId;
      messageContent = other.messageContent;
      setPacketInformation(other);
   }

   public TopicID getTopicId()
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
