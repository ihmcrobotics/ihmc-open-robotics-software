package us.ihmc.javaFXToolkit.messager;

import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TopicID;

/**
 * A message can be used with a {@link Messager}.
 * 
 * @author Sylvain Bertrand
 *
 * @param <T> the type of data this message carries.
 */
public final class Message<T>
{
   /**
    * The ID of the topic the data is for.
    * <p>
    * This field is public and non-final only for serialization purposes, it is not meant to be
    * accessed directly.
    * </p>
    */
   public TopicID topicID;
   /**
    * The data this message carries.
    * <p>
    * This field is public and non-final only for serialization purposes, it is not meant to be
    * accessed directly.
    * </p>
    */
   public T messageContent;

   /** Empty constructor only used for serialization purposes. */
   public Message()
   {
   }

   /**
    * Creates a new message given a the data to carry for a given topic.
    * 
    * @param topic the topic the data is for.
    * @param messageContent the data to carry.
    */
   public Message(Topic<T> topic, T messageContent)
   {
      this.topicID = topic.getUniqueID();
      this.messageContent = messageContent;
   }

   /**
    * Creates a new message given a the data to carry for a given topic.
    * 
    * @param topicID the ID of the topic the data is for.
    * @param messageContent the data to carry.
    */
   public Message(TopicID topicID, T messageContent)
   {
      this.topicID = topicID;
      this.messageContent = messageContent;
   }

   /**
    * Copy constructor.
    * 
    * @param other the other message to copy.
    */
   public Message(Message<T> other)
   {
      set(other);
   }

   /**
    * Copy setter.
    * 
    * @param other the other message to copy.
    */
   public void set(Message<T> other)
   {
      topicID = other.topicID;
      messageContent = other.messageContent;
   }

   /**
    * Gets the ID of the topic this message is for.
    * 
    * @return the topic ID.
    */
   public TopicID getTopicID()
   {
      return topicID;
   }

   /**
    * Retrieves the topic from the given API this message is for.
    * 
    * @param api the API that contains the topic of this message.
    * @return the topic this message is for.
    */
   public Topic<T> getTopic(MessagerAPI api)
   {
      return api.findTopic(topicID);
   }

   /**
    * Gets the data this message is carrying.
    * 
    * @return the data this message is carrying.
    */
   public T getMessageContent()
   {
      return messageContent;
   }

   public boolean epsilonEquals(Message<T> other, double epsilon)
   {
      return topicID.equals(other.topicID) && messageContent.equals(other.messageContent);
   }
}
