package us.ihmc.robotEnvironmentAwareness.communication;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetworkedObjectCommunicator;
import us.ihmc.communication.net.local.IntraprocessObjectCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Message;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerStateListener;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.TopicListenerBase;

/**
 * @deprecated This is an old implementation that imposes restrictions like having a NetClassList.
 * Use {@link us.ihmc.messager.kryo.KryoMessager} instead.
 */
public class KryoMessager implements Messager
{
   public static final int BUFFER_SIZE = 2097152 * 20;
   private static final boolean DEBUG = false;

   private final MessagerAPI messagerAPI;

   private final ConcurrentHashMap<Topic<?>, List<AtomicReference<Object>>> inputVariablesMap = new ConcurrentHashMap<>();
   private final ConcurrentHashMap<Topic<?>, List<TopicListenerBase<Object>>> topicListenersMap = new ConcurrentHashMap<>();
   private final List<MessagerStateListener> messagerStateListeners = new ArrayList<>();

   /**
    * When {@code true}, then a call to {@link #submitMessage(Message)} on this messager will also ensure the local inputs and listeners are receiving the message.
    * When {@code false}, {@link #submitMessage(Message)} will only submit the message to the other endpoint.
    */
   private boolean allowSelfSubmit = false;
   private final NetworkedObjectCommunicator objectCommunicator;

   public static KryoMessager createTCPServer(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      NetworkedObjectCommunicator communicator = new KryoObjectServer(port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
      return new KryoMessager(messagerAPI, communicator);
   }

   public static KryoMessager createTCPClient(MessagerAPI messagerAPI, String host, NetworkPorts port, NetClassList netClassList)
   {
      KryoObjectClient objectCommunicator = new KryoObjectClient(KryoObjectClient.getByName(host), port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
      objectCommunicator.setReconnectAutomatically(true);
      return new KryoMessager(messagerAPI, objectCommunicator);
   }

   public static KryoMessager createIntraprocess(MessagerAPI messagerAPI, NetworkPorts port, NetClassList netClassList)
   {
      return new KryoMessager(messagerAPI, new IntraprocessObjectCommunicator(port.getPort(), netClassList));
   }

   private KryoMessager(MessagerAPI messagerAPI, NetworkedObjectCommunicator objectCommunicator)
   {
      this.messagerAPI = messagerAPI;
      this.objectCommunicator = objectCommunicator;
      this.objectCommunicator.attachListener(Message.class, this::receiveREAMessage);
   }

   /**
    * When {@code true}, then a call to {@link #submitMessage(Message)} on this messager will also ensure the local inputs and listeners are receiving the message.
    * When {@code false}, {@link #submitMessage(Message)} will only submit the message to the other endpoint.
    */
   public void setAllowSelfSubmit(boolean allowSelfSubmit)
   {
      this.allowSelfSubmit = allowSelfSubmit;
   }

   @SuppressWarnings("unchecked")
   private <T> void receiveREAMessage(Message<T> message)
   {
      if (message == null)
         return;

      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<T> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (DEBUG)
         LogTools.info("Packet received from network with message name: " + messageTopic.getName());

      List<AtomicReference<Object>> inputVariablesForTopic = inputVariablesMap.get(messageTopic);
      if (inputVariablesForTopic != null)
         inputVariablesForTopic.forEach(variable -> variable.set(message.getMessageContent()));

      List<TopicListenerBase<Object>> topicListeners = topicListenersMap.get(messageTopic);
      if (topicListeners != null)
         topicListeners.forEach(listener -> listener.receivedMessageForTopic((Message<Object>) message));
   }

   @Override
   public <T> void submitMessage(Message<T> message)
   {
      if (!messagerAPI.containsTopic(message.getTopicID()))
         throw new RuntimeException("The message is not part of this messager's API.");

      Topic<?> messageTopic = messagerAPI.findTopic(message.getTopicID());

      if (allowSelfSubmit)
         receiveREAMessage(message);

      if (!objectCommunicator.isConnected())
      {
         LogTools.warn("This messager is closed, message's topic: " + messageTopic.getName());
         return;
      }

      if (DEBUG)
         LogTools.info("Submit message for topic: " + messageTopic.getName());

      // Variable update over network
      objectCommunicator.send(message);
   }

   /** {@inheritDoc} */
   @Override
   public <T> AtomicReference<T> createInput(Topic<T> topic, T defaultValue)
   {
      AtomicReference<T> boundVariable = new AtomicReference<>(defaultValue);
      attachInput(topic, boundVariable);
      return boundVariable;
   }

   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public <T> void attachInput(Topic<T> topic, AtomicReference<T> input)
   {
      List<AtomicReference<Object>> boundVariablesForTopic = inputVariablesMap.computeIfAbsent(topic, k -> new ArrayList<>());
      boundVariablesForTopic.add((AtomicReference<Object>) input);
   }

   /** {@inheritDoc} */
   @Override
   public <T> boolean removeInput(Topic<T> topic, AtomicReference<T> input)
   {
      List<?> boundVariablesForTopic = inputVariablesMap.get(topic);
      if (boundVariablesForTopic == null)
         return false;
      else
         return boundVariablesForTopic.remove(input);
   }

   @Override
   @SuppressWarnings("unchecked")
   public <T> void addTopicListenerBase(Topic<T> topic, TopicListenerBase<T> listener)
   {
      List<TopicListenerBase<Object>> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
      {
         topicListeners = new ArrayList<>();
         topicListenersMap.put(topic, topicListeners);
      }
      topicListeners.add((TopicListener<Object>) listener);
   }

   /** {@inheritDoc} */
   @Override
   public <T> boolean removeTopicListener(Topic<T> topic, TopicListenerBase<T> listener)
   {
      List<?> topicListeners = topicListenersMap.get(topic);
      if (topicListeners == null)
         return false;
      else
         return topicListeners.remove(listener);
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
   public void addMessagerStateListener(MessagerStateListener listener)
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
   public boolean removeMessagerStateListener(MessagerStateListener listener)
   {
      throw new UnsupportedOperationException("Unsupported operation due to API restriction of NetworkedObjectCommunicator.");
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
