package us.ihmc.javaFXToolkit.messager;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;

/**
 * Implementation of {@code Messager} using shared memory.
 * 
 * @author Sylvain Bertrand
 */
public class SharedMemoryMessager implements Messager
{
   private final MessagerAPI messagerAPI;

   private final AtomicBoolean isConnected = new AtomicBoolean(false);
   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<MessagerStateListener> connectionStateListeners = new ArrayList<>();

   /**
    * Creates a new messager.
    * 
    * @param messagerAPI the API to use with this messager.
    */
   public SharedMemoryMessager(MessagerAPI messagerAPI)
   {
      this.messagerAPI = messagerAPI;
   }

   /** {@inheritDoc} */
   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<?> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (!isConnected.get())
      {
         PrintTools.warn(this, "This messager is closed, message's topic: " + messageTopic.getSimpleName());
         return;
      }

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(messageTopic);
      if (boundVariablesForTopic != null)
         boundVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<TopicListener<Object>> topicListeners = topicListenersMap.get(messageTopic);
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic(message.getMessageContent()));
   }

   /** {@inheritDoc} */
   @Override
   @SuppressWarnings("unchecked")
   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(topic);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         boundVariables.put(topic, boundVariablesForTopic);
      }
      boundVariablesForTopic.add((AtomicReference<Object>) boundVariable);
      return boundVariable;
   }

   /** {@inheritDoc} */
   @Override
   @SuppressWarnings("unchecked")
   public <T> void registerTopicListener(Topic<T> topic, TopicListener<T> listener)
   {
      List<TopicListener<Object>> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         topicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((TopicListener<Object>) listener);
   }

   /** {@inheritDoc} */
   @Override
   public void startMessager()
   {
      isConnected.set(true);
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(true));
   }

   /** {@inheritDoc} */
   @Override
   public void closeMessager()
   {
      isConnected.set(false);
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(false));
      boundVariables.clear();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isMessagerOpen()
   {
      return isConnected.get();
   }

   /** {@inheritDoc} */
   @Override
   public void registerMessagerStateListener(MessagerStateListener listener)
   {
      connectionStateListeners.add(listener);
   }

   /** {@inheritDoc} */
   @Override
   public void notifyMessagerStateListeners()
   {
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(isMessagerOpen()));
   }

   /** {@inheritDoc} */
   @Override
   public MessagerAPI getMessagerAPI()
   {
      return messagerAPI;
   }
}
