package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetworkedObjectCommunicator;
import us.ihmc.communication.net.local.IntraprocessObjectCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.Message;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.Topic;
import us.ihmc.javaFXToolkit.messager.MessagerStateListener;
import us.ihmc.javaFXToolkit.messager.TopicListener;

public class KryoMessager implements Messager
{
   public static final int BUFFER_SIZE = 2097152 * 20;
   private static final boolean DEBUG = false;

   private final MessagerAPI messagerAPI;

   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListener<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<MessagerStateListener> messagerStateListeners = new ArrayList<>();

   private final NetworkedObjectCommunicator objectCommunicator;

   public static Messager createTCPServer(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      NetworkedObjectCommunicator communicator = new KryoObjectServer(port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
      return new KryoMessager(messagerAPI, communicator);
   }

   public static Messager createTCPClient(MessagerAPI messagerAPI, String host, NetworkPorts port, NetClassList netClassList)
   {
      KryoObjectClient objectCommunicator = new KryoObjectClient(KryoObjectClient.getByName(host), port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
      objectCommunicator.setReconnectAutomatically(true);
      return new KryoMessager(messagerAPI, objectCommunicator);
   }

   public static Messager createIntraprocess(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      return new KryoMessager(messagerAPI, new IntraprocessObjectCommunicator(port.getPort(), netClassList));
   }

   private KryoMessager(MessagerAPI messagerAPI, NetworkedObjectCommunicator objectCommunicator)
   {
      this.messagerAPI = messagerAPI;
      this.objectCommunicator = objectCommunicator;
      this.objectCommunicator.attachListener(Message.class, this::receiveREAMessage);
   }

   private <T> void receiveREAMessage(Message<T> message)
   {
      if (message == null)
         return;

      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<T> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (DEBUG)
         PrintTools.info("Packet received from network with message name: " + messageTopic.getName());

      List<AtomicReference<Object>> inputVariablesForTopic = inputVariablesMap.get(messageTopic);
      if (inputVariablesForTopic != null)
         inputVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<TopicListener<Object>> topicListeners = topicListenersMap.get(messageTopic);
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic(message.getMessageContent()));
   }

   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<?> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (!objectCommunicator.isConnected())
      {
         PrintTools.warn(this, "This messager is closed, message's topic: " + messageTopic.getName());
         return;
      }

      if (DEBUG)
         PrintTools.info("Submit message for topic: " + messageTopic.getName());

      // Variable update over network
      objectCommunicator.send(message);
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);

      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.get(topic);
      if (boundVariablesForTopic == null)
      {
         boundVariablesForTopic = new ArrayList<>();
         inputVariablesMap.put(topic, boundVariablesForTopic);
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
      objectCommunicator.connect();
   }

   @Override
   public void closeMessager() throws IOException
   {
      inputVariablesMap.clear();
      objectCommunicator.closeConnection();
      objectCommunicator.disconnect();
   }

   @Override
   public boolean isMessagerOpen()
   {
      return objectCommunicator.isConnected();
   }

   @Override
   public void registerMessagerStateListener(MessagerStateListener listener)
   {
      objectCommunicator.attachStateListener(new ConnectionStateListener()
      {
         @Override
         public void disconnected()
         {
            listener.messagerStateChanged(false);
         }

         @Override
         public void connected()
         {
            listener.messagerStateChanged(true);
         }
      });
      messagerStateListeners.add(listener);
   }

   @Override
   public void notifyMessagerStateListeners()
   {
      messagerStateListeners.forEach(listener -> listener.messagerStateChanged(isMessagerOpen()));
   }

   @Override
   public MessagerAPI getMessagerAPI()
   {
      return messagerAPI;
   }
}
