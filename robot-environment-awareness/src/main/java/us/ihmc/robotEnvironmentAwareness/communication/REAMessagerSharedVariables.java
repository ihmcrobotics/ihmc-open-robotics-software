package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.Message;
import us.ihmc.javaFXToolkit.messager.MessagerStateListener;
import us.ihmc.javaFXToolkit.messager.TopicListener;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;

public class REAMessagerSharedVariables implements Messager
{
   private final MessagerAPI messagerAPI;

   private final AtomicBoolean isConnected = new AtomicBoolean(false);
   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<MessagerStateListener> connectionStateListeners = new ArrayList<>();

   public REAMessagerSharedVariables(MessagerAPI messagerAPI)
   {
      this.messagerAPI = messagerAPI;
   }

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

   @Override
   public void startMessager() throws IOException
   {
      isConnected.set(true);
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(true));
   }

   @Override
   public void closeMessager()
   {
      isConnected.set(false);
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(false));
      boundVariables.clear();
   }

   @Override
   public boolean isMessagerOpen()
   {
      return isConnected.get();
   }

   @Override
   public void registerMessagerStateListener(MessagerStateListener listener)
   {
      connectionStateListeners.add(listener);
   }

   @Override
   public void notifyMessagerStateListeners()
   {
      connectionStateListeners.forEach(listener -> listener.messagerStateChanged(isMessagerOpen()));
   }

   @Override
   public MessagerAPI getMessagerAPI()
   {
      return messagerAPI;
   }
}
