package us.ihmc.communication;

/**
 * Generator to automatically generate a topic name based on the type of message to send.
 */
@Deprecated
public interface MessageTopicNameGenerator
{
   @Deprecated
   String generateTopicName(Class<?> messageType);
}
