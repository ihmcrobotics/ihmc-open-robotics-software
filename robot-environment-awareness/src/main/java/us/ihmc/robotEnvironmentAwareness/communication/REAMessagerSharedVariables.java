package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.API;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;

public class REAMessagerSharedVariables implements REAMessager
{
   private final API messagerAPI;

   private final AtomicBoolean isConnected = new AtomicBoolean(false);
   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> boundVariables = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<REATopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<ConnectionStateListener> connectionStateListeners = new ArrayList<>();

   public REAMessagerSharedVariables(API messagerAPI)
   {
      this.messagerAPI = messagerAPI;
   }

   @Override
   public <T> void submitMessage(REAMessage<T> message)
   {
      if (!messagerAPI.containsTopic(message.getTopicId()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<?> messageTopic = messagerAPI.findTopic(message.getTopicId());

      if (!isConnected.get())
      {
         PrintTools.warn(this, "This messager is closed, message's topic: " + messageTopic.getSimpleName());
         return;
      }

      List<AtomicReference<Object>> boundVariablesForTopic = boundVariables.get(messageTopic);
      if (boundVariablesForTopic != null)
         boundVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(messageTopic);
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
   public <T> void registerTopicListener(Topic<T> topic, REATopicListener<T> listener)
   {
      List<REATopicListener<Object>> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         topicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((REATopicListener<Object>) listener);
   }

   @Override
   public void startMessager() throws IOException
   {
      isConnected.set(true);
      connectionStateListeners.forEach(ConnectionStateListener::connected);
   }

   @Override
   public void closeMessager()
   {
      isConnected.set(false);
      connectionStateListeners.forEach(ConnectionStateListener::disconnected);
      boundVariables.clear();
   }

   @Override
   public boolean isMessagerOpen()
   {
      return isConnected.get();
   }

   @Override
   public void registerConnectionStateListener(ConnectionStateListener listener)
   {
      connectionStateListeners.add(listener);
   }

   @Override
   public void notifyConnectionStateListeners()
   {
      if (isMessagerOpen())
         connectionStateListeners.forEach(ConnectionStateListener::connected);
      else
         connectionStateListeners.forEach(ConnectionStateListener::disconnected);
   }

   @Override
   public API getMessagerAPI()
   {
      return messagerAPI;
   }
}
