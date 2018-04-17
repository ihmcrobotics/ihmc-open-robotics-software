package us.ihmc.robotEnvironmentAwareness.communication;

public interface REATopicListener<T>
{
   public void receivedMessageForTopic(T messageContent);
}
